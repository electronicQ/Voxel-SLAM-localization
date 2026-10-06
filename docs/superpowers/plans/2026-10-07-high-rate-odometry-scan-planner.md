# Voxel-SLAM High-Rate Odometry and ROS1 SCAN-Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an IMU-rate pose predictor and ROS1 navigation adapter so Voxel-SLAM supplies `/LIO/odom_imu`, high-rate `camera_init -> aft_mapped`, and `/LIO/odom_vehicle` without changing the authoritative scan-batched IESKF.

**Architecture:** A pure C++ `ImuRatePredictor` owns a posterior snapshot, an IMU tail, and an incrementally propagated mean state. A dedicated ROS callback queue feeds both the existing authoritative IMU buffer and this predictor; a separate adapter node composes the IMU odometry with a URDF static transform to publish vehicle odometry.

**Tech Stack:** ROS1 Noetic, C++14, Eigen3, `nav_msgs/Odometry`, `sensor_msgs/Imu`, ROS `tf`, catkin/gtest.

## Global Constraints

- Preserve the scan-batched `IMUEKF::process()` path, covariance propagation, and point-cloud deskew.
- Keep sliding-window BA in mapping and pre-relocalization `LOCAL_ODOMETRY`; do not add BA to `IN_MAP_TRACKING`.
- Publish navigation odometry in localization mode only while localization is valid unless explicitly configured otherwise.
- Use sensor timestamps, never wall-clock time, for odometry and dynamic TF.
- Keep robot installation extrinsics outside Voxel-SLAM and consume them through TF in a separate adapter node.
- Do not publish a world-frame cloud under the sensor-frame `/LIO/clouds_lidar` name.
- This Windows host cannot prove ROS Noetic compilation or runtime behavior; preserve Ubuntu verification commands and do not report them as executed here.

---

### Task 1: Test-first pure IMU-rate predictor

**Files:**
- Create: `VoxelSLAM/src/imu_rate_predictor.hpp`
- Create: `VoxelSLAM/src/imu_rate_predictor.cpp`
- Create: `VoxelSLAM/test/imu_rate_predictor_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `IMUST` and `Exp()` from `VoxelSLAM/src/tools.hpp`.
- Produces: `ImuRatePredictor`, `ImuRateSample`, `ImuRatePrediction`, and `ImuRateRejectReason`.

- [ ] **Step 1: Write the failing predictor tests**

Create tests that call this API:

```cpp
ImuRatePredictor predictor(0.5);
predictor.updateSnapshot(posterior, 1.0);
ImuRatePrediction prediction = predictor.pushImu(sample);
EXPECT_TRUE(prediction.publishable);
EXPECT_NEAR(prediction.state.p.x(), expected_x, 1e-9);
```

Cover gravity-balanced stationary propagation, constant velocity, constant yaw rate, snapshot replacement with tail replay, reset, non-monotonic timestamps, non-finite input, and `max_prediction_sec` rejection.

- [ ] **Step 2: Verify RED on Ubuntu**

Run:

```bash
catkin_make --pkg voxel_slam --make-args imu_rate_predictor_test
```

Expected: compilation fails because `imu_rate_predictor.hpp` and its symbols do not exist.

- [ ] **Step 3: Implement the minimal predictor**

Use the exact public API:

```cpp
struct ImuRateSample {
  double t = 0.0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

enum class ImuRateRejectReason {
  NONE,
  DISABLED,
  NO_SNAPSHOT,
  NONFINITE,
  NON_MONOTONIC_TIME,
  PREDICTION_HORIZON_EXCEEDED
};

struct ImuRatePrediction {
  bool publishable = false;
  IMUST state;
  Eigen::Vector3d unbiased_angular_velocity = Eigen::Vector3d::Zero();
  ImuRateRejectReason reason = ImuRateRejectReason::NO_SNAPSHOT;
};

class ImuRatePredictor {
 public:
  explicit ImuRatePredictor(double max_prediction_sec = 0.5);
  void setEnabled(bool enabled);
  void setMaxPredictionSec(double seconds);
  void updateSnapshot(const IMUST &posterior, double acceleration_scale);
  ImuRatePrediction pushImu(const ImuRateSample &sample);
  void reset();
};
```

Protect all mutable state with a mutex. Keep a bounded deque of samples newer than the snapshot, propagate one new sample incrementally, and replay the retained tail exactly once after `updateSnapshot()`.

- [ ] **Step 4: Verify GREEN on Ubuntu**

Run:

```bash
catkin_make --pkg voxel_slam --make-args imu_rate_predictor_test
./devel/lib/voxel_slam/imu_rate_predictor_test
```

Expected: all predictor tests pass.

- [ ] **Step 5: Commit**

```bash
git add VoxelSLAM/src/imu_rate_predictor.* VoxelSLAM/test/imu_rate_predictor_test.cpp VoxelSLAM/CMakeLists.txt
git commit -m "feat: add IMU-rate state predictor"
```

### Task 2: Integrate the predictor with a dedicated ROS IMU callback queue

**Files:**
- Modify: `VoxelSLAM/src/voxelslam.cpp`
- Modify: `VoxelSLAM/src/voxelslam.hpp`
- Modify: `VoxelSLAM/config/localization_mid360.yaml`
- Modify: `VoxelSLAM/config/mid360.yaml`
- Modify: `VoxelSLAM/config/avia.yaml`
- Modify: `VoxelSLAM/config/avia_fly.yaml`
- Modify: `VoxelSLAM/config/hesai.yaml`
- Modify: `VoxelSLAM/config/velodyne.yaml`

**Interfaces:**
- Consumes: `ImuRatePredictor` from Task 1 and the existing `localization_valid`, `x_curr`, `odom_ekf.scale_gravity`, `system_reset()`, `apply_frontend_global_correction()`, and `lio_state_estimation()` paths.
- Produces: `/LIO/odom_imu` plus high-rate `camera_init -> aft_mapped` TF.

- [ ] **Step 1: Add a failing source-contract test**

Create `VoxelSLAM/test/high_rate_odom_contract_test.py` that reads `voxelslam.cpp` and asserts all of these tokens exist together:

```python
required = [
    'ros::CallbackQueue imu_callback_queue',
    'ros::AsyncSpinner',
    '"/LIO/odom_imu"',
    'update_imu_rate_snapshot',
    'reset_imu_rate_output',
    'prediction.state.t',
]
```

It must also assert that high-rate TF stamping does not use `ros::Time::now()`.

- [ ] **Step 2: Verify RED on Windows**

Run:

```powershell
python VoxelSLAM/test/high_rate_odom_contract_test.py
```

Expected: FAIL because the dedicated queue, publisher, and snapshot hooks are absent.

- [ ] **Step 3: Add dedicated IMU callback ownership**

Move the IMU callback into `VOXEL_SLAM` and create members equivalent to:

```cpp
ros::CallbackQueue imu_callback_queue;
std::unique_ptr<ros::AsyncSpinner> imu_spinner;
ros::Publisher pub_imu_odom;
std::unique_ptr<ImuRatePredictor> imu_rate_predictor;
tf::TransformBroadcaster imu_rate_tf_broadcaster;
bool imu_rate_output_enabled = true;
bool publish_before_localized = false;
std::string imu_odom_topic = "/LIO/odom_imu";
std::string odom_frame = "camera_init";
std::string imu_child_frame = "aft_mapped";
```

Subscribe through a NodeHandle assigned to `imu_callback_queue`, start one `ros::AsyncSpinner`, and keep LiDAR callbacks on the original queue. The callback must first append to `imu_buf` under `mBuf`, then call `pushImu()` and publish only when mode/state gating permits.

- [ ] **Step 4: Publish odometry and TF with sensor timestamps**

Construct `nav_msgs::Odometry` from `ImuRatePrediction`:

```cpp
odom.header.stamp.fromSec(prediction.state.t);
odom.header.frame_id = odom_frame;
odom.child_frame_id = imu_child_frame;
odom.pose.pose.position.x = prediction.state.p.x();
odom.twist.twist.linear.x = prediction.state.v.x();
odom.twist.twist.angular.x = prediction.unbiased_angular_velocity.x();
```

Broadcast the same transform using the same stamp. Change scan-rate `ResultOutput::pub_odom_func()` to return without broadcasting whenever the high-rate publisher owns the TF and has a snapshot; retain scan-rate fallback when high-rate output is disabled.

- [ ] **Step 5: Refresh and reset snapshots at all state boundaries**

Call `update_imu_rate_snapshot(x_curr)` after successful scan IESKF updates, after BA writes the newest state into `x_curr`, and after `apply_frontend_global_correction()` succeeds. Call `reset_imu_rate_output()` from `system_reset()`, LOST cleanup, time rollback handling, and process shutdown. In `IN_MAP_TRACKING`, only scan IESKF refreshes the snapshot because BA is skipped.

- [ ] **Step 6: Add YAML parameters**

Add this exact block under every existing `Odometry:` section, preserving sensor-specific values:

```yaml
  imu_rate_output: true
  imu_odom_topic: "/LIO/odom_imu"
  odom_frame: "camera_init"
  imu_child_frame: "aft_mapped"
  publish_before_localized: false
  max_prediction_sec: 0.5
```

- [ ] **Step 7: Verify available checks**

Run on Windows:

```powershell
python VoxelSLAM/test/high_rate_odom_contract_test.py
git diff --check
```

Run later on Ubuntu:

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
```

Expected: Windows contract and diff checks pass; Ubuntu build and tests pass when executed there.

- [ ] **Step 8: Commit**

```bash
git add VoxelSLAM/src/voxelslam.cpp VoxelSLAM/src/voxelslam.hpp VoxelSLAM/config VoxelSLAM/test/high_rate_odom_contract_test.py
git commit -m "feat: publish IMU-rate localization odometry"
```

### Task 3: Test-first vehicle odometry transform and ROS adapter

**Files:**
- Create: `VoxelSLAM/src/vehicle_odom_transform.hpp`
- Create: `VoxelSLAM/src/vehicle_odom_transform.cpp`
- Create: `VoxelSLAM/src/scan_planner_odom_adapter.cpp`
- Create: `VoxelSLAM/test/vehicle_odom_transform_test.cpp`
- Create: `VoxelSLAM/config/scan_planner_adapter.yaml`
- Create: `VoxelSLAM/launch/scan_planner_adapter.launch`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `/LIO/odom_imu`, `/localization/valid`, and static TF `imu_frame -> base_frame`.
- Produces: `/LIO/odom_vehicle` with world-frame linear velocity for ROS1 SCAN-Planner.

- [ ] **Step 1: Write failing transform tests**

Test the pure function:

```cpp
VehicleOdomState composeVehicleOdom(
    const VehicleOdomState &imu,
    const Eigen::Isometry3d &T_imu_base);
```

Cover identity extrinsic, translated base pose, rotated extrinsic, and lever-arm velocity `v_base = v_imu + omega_world.cross(R_odom_imu * t_imu_base)`.

- [ ] **Step 2: Verify RED on Ubuntu**

Run:

```bash
catkin_make --pkg voxel_slam --make-args vehicle_odom_transform_test
```

Expected: compile failure because the transform module does not exist.

- [ ] **Step 3: Implement the pure transform and pass tests**

Define:

```cpp
struct VehicleOdomState {
  double t = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_body = Eigen::Vector3d::Zero();
};
```

Compose pose and lever-arm velocity without ROS dependencies, then link it into `voxel_slam_map_io` and its gtest.

- [ ] **Step 4: Implement the ROS adapter node**

Subscribe to configured IMU odometry and validity topics. Cache one static TF using `tf::TransformListener`; on each valid odometry message, convert to `VehicleOdomState`, call `composeVehicleOdom()`, and publish `/LIO/odom_vehicle`. Do not publish dynamic TF. Use throttled warnings for missing TF, invalid state, invalid localization, and input timeout.

- [ ] **Step 5: Add configuration and launch**

Use these defaults:

```yaml
imu_odom_topic: "/LIO/odom_imu"
vehicle_odom_topic: "/LIO/odom_vehicle"
imu_frame: "aft_mapped"
base_frame: "base_link"
localization_valid_topic: "/localization/valid"
require_localization_valid: true
tf_timeout_sec: 0.2
input_timeout_sec: 0.2
```

The launch file loads this YAML and starts only `scan_planner_odom_adapter`.

- [ ] **Step 6: Verify and commit**

Run later on Ubuntu:

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
```

Run now on Windows:

```powershell
git diff --check
```

Commit:

```bash
git add VoxelSLAM/src/vehicle_odom_transform.* VoxelSLAM/src/scan_planner_odom_adapter.cpp VoxelSLAM/test/vehicle_odom_transform_test.cpp VoxelSLAM/config/scan_planner_adapter.yaml VoxelSLAM/launch/scan_planner_adapter.launch VoxelSLAM/CMakeLists.txt
git commit -m "feat: adapt IMU odometry for ROS1 SCAN-Planner"
```

### Task 4: Document SCAN-Planner wiring and complete static verification

**Files:**
- Modify: `README.md`
- Modify: `HANDOFF.md`
- Modify: `VoxelSLAM/README.md`

**Interfaces:**
- Consumes: all runtime interfaces from Tasks 1-3.
- Produces: exact launch, parameter, topic, TF, and Ubuntu validation instructions.

- [ ] **Step 1: Document dataflow and safety gates**

Document that `/LIO/odom_imu` is a high-rate prediction from the latest posterior, `/LIO/odom_vehicle` comes from the TF adapter, continuous localization uses IESKF only, and pre-relocalization `LOCAL_ODOMETRY` retains BA.

- [ ] **Step 2: Document SCAN parameters**

Record these exact values:

```text
body_pose_topic=/LIO/odom_vehicle
sensor_pose_topic=/LIO/odom_imu
cloud_topic=/map_scan
grid_map/frame_id=camera_init
grid_map/cloud_is_world=true
grid_map/need_extrinsic=false
```

- [ ] **Step 3: Run final checks**

Run on Windows:

```powershell
python VoxelSLAM/test/evaluate_localization_test.py
python VoxelSLAM/test/high_rate_odom_contract_test.py
git diff --check
git status --short
```

Run later on Ubuntu:

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
rostopic hz /LIO/odom_imu
rostopic hz /LIO/odom_vehicle
rostopic delay /LIO/odom_imu
rosrun tf tf_echo camera_init aft_mapped
```

- [ ] **Step 4: Commit documentation**

```bash
git add README.md HANDOFF.md VoxelSLAM/README.md
git commit -m "docs: explain SCAN-Planner odometry integration"
```

- [ ] **Step 5: Review the complete branch**

Review from design base `9b355f9` through `HEAD`, checking thread lifetime, lock order, state-machine gates, timestamp provenance, TF ownership, ROS1 message semantics, and completeness of Ubuntu-only verification notes.
