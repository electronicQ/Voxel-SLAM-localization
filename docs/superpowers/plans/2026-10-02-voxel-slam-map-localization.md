# Voxel-SLAM Map Persistence and Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add versioned map-package export, persisted BTC descriptors, global or position-bounded BTC relocalization, GTSAM session alignment, read-only historical-map IESKF tracking, RViz direct initialization, and automatic LOST recovery to Voxel-SLAM.

**Architecture:** Mapping mode records one fused keyframe and one BTC frame per accepted motion threshold, then exports final optimized poses and local-frame clouds as a versioned map package. Localization mode loads that package into immutable historical data, runs temporary local odometry until BTC or RViz supplies a map alignment, applies one global correction, and tracks against dynamically loaded historical keyframes without inserting current scans. New functionality lives in focused libraries; `voxelslam.cpp` owns ROS wiring and state dispatch only.

**Tech Stack:** Ubuntu 20.04, ROS Noetic, C++14, PCL 1.10, Eigen 3.3.7, GTSAM 4.0.3, Boost.Filesystem, catkin gtest, Livox ROS Driver, existing Voxel-SLAM BTC and LiDAR-IMU IESKF.

## Global Constraints

- Modify only this Voxel-SLAM repository; do not copy Lumi-Nav code.
- Sensors remain LiDAR and IMU only.
- Use the name BTC, never BCT.
- Localization history is immutable: no historical pose, cloud, descriptor, or map-package write-back.
- One accepted multi-Scan fused keyframe maps to exactly one BTC descriptor frame.
- Mapping and localization use identical `win_size`, motion thresholds, preprocessing, BTC settings, and frame conventions.
- Export only after final loop/graph optimization, using final `T_map_keyframe` poses and keyframe-local clouds.
- `icp_normal()` is the only extra loop/refinement ICP; do not add a duplicate point-to-plane GN stage.
- Keep GTSAM/iSAM2; do not add SE-Sync.
- BTC/ICP transform direction must be established by synthetic tests as `p_H = R_HC * p_C + t_HC` before rosbag use.
- Windows may perform source checks only. ROS build, catkin tests, rosbag, and vehicle validation require Ubuntu 20.04.
- Every task commits only its own files after its tests/checks; unrelated user changes must remain untouched.

---

## File Structure

**Create**

- `VoxelSLAM/src/map_package.hpp/.cpp`: immutable saved-map model, manifest/CSV/PCD IO, fingerprinting, atomic directory publication.
- `VoxelSLAM/src/btc_io.hpp/.cpp`: stable BTC binary codec and `STDescManager` restoration.
- `VoxelSLAM/src/localization_state.hpp`: states, alignment source, quality counters, versioned RViz request.
- `VoxelSLAM/src/prior_map_tracker.hpp/.cpp`: XY/XYZ keyframe indexes, local historical-map cache, quality accounting.
- `VoxelSLAM/src/relocalizer.hpp/.cpp`: global/bounded BTC search, radius expansion, ICP verification.
- `VoxelSLAM/src/session_aligner.hpp/.cpp`: fixed-history-anchor GTSAM graph and `T_W_O` calculation.
- `VoxelSLAM/src/global_correction.hpp/.cpp`: pure correction helpers used by `VOXEL_SLAM`.
- `VoxelSLAM/test/*.cpp`: focused unit tests listed per task.
- `VoxelSLAM/config/localization_mid360.yaml`: localization defaults.
- `VoxelSLAM/launch/vxlm_localization_mid360.launch`: localization launch entry.
- `VoxelSLAM/scripts/evaluate_localization.py`: offline ATE/RPE and map-hash report.

**Modify**

- `VoxelSLAM/src/BTC.h/.cpp`: frame-filtered search and explicit cross-session candidate selection.
- `VoxelSLAM/src/voxelslam.cpp`: mapping capture/export, map load, state dispatch, ROS topics, frontend handoff.
- `VoxelSLAM/CMakeLists.txt`: focused libraries, executable links, catkin tests.
- `VoxelSLAM/package.xml`: explicit runtime/build/test dependencies already consumed by CMake and ROS messages.
- `VoxelSLAM/config/mid360.yaml`: mapping-mode keys and export path.
- `VoxelSLAM/README.md`: map build/localization workflow and validation limitations.

---

### Task 1: Build Targets and Saved-Map Data Model

**Files:**
- Create: `VoxelSLAM/src/map_package.hpp`
- Create: `VoxelSLAM/src/map_package.cpp`
- Create: `VoxelSLAM/test/map_package_roundtrip_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`
- Modify: `VoxelSLAM/package.xml`

**Interfaces:**
- Produces: `SavedPose`, `SavedKeyframe`, `SavedBtcFrame`, `SavedMap`, `MapPackageOptions`, keyframe-only `MapPackageIO::save/load`, `btc_config_fingerprint`. Task 1 round-trips maps whose BTC frame vector is empty; Task 2 extends the package transaction with BTC files.
- Consumes: `PointType`, `STD`, `ConfigSetting` from existing headers.

- [ ] **Step 1: Add the failing round-trip and malformed-input tests**

Create tests that build two local-frame point clouds, known poses, and manifest options, then assert exact IDs/poses and point tolerances after save/load. Add separate tests for wrong version, missing PCD, non-contiguous keyframe IDs, and mismatched fingerprint.

```cpp
TEST(MapPackageIO, RoundTripsKeyframes) {
  SavedMap input = makeTwoKeyframeMap();
  ASSERT_TRUE(MapPackageIO::save(temp.path(), input, options(), error)) << error;
  SavedMap output;
  ASSERT_TRUE(MapPackageIO::load(temp.path(), output, options(), error)) << error;
  ASSERT_EQ(output.keyframes.size(), 2u);
  EXPECT_EQ(output.keyframes[1].id, 1);
  EXPECT_TRUE(output.keyframes[1].pose.t.isApprox(input.keyframes[1].pose.t, 1e-9));
}
```

- [ ] **Step 2: Register a reusable core library and the test target**

```cmake
find_package(Boost REQUIRED COMPONENTS filesystem system)
add_library(voxel_slam_map_io src/map_package.cpp)
target_link_libraries(voxel_slam_map_io ${PCL_LIBRARIES} ${Boost_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(map_package_roundtrip_test test/map_package_roundtrip_test.cpp)
  target_link_libraries(map_package_roundtrip_test voxel_slam_map_io ${catkin_LIBRARIES})
endif()
```

- [ ] **Step 3: Run the test to confirm the missing API fails on Ubuntu**

Run: `catkin_make run_tests_voxel_slam_gtest_map_package_roundtrip_test`

Expected: compile failure naming undefined saved-map types/functions. On Windows record `NOT RUN: ROS Noetic toolchain unavailable` and run `rg -n "MapPackageIO|SavedMap" VoxelSLAM/test/map_package_roundtrip_test.cpp`.

- [ ] **Step 4: Implement strict manifest, CSV, and PCD IO**

Use fixed-schema scalar parsing rather than a general YAML dependency. Write `manifest.yaml`, `keyframes.csv`, and keyframe-local binary PCDs into a unique sibling temporary directory. Validate magic fields, version `1`, frame names, descriptor policy, finite poses, continuous IDs, counts, and cloud existence. Publish with `boost::filesystem::rename` only after every write succeeds; reject an existing final destination rather than overwriting it.

```cpp
class MapPackageIO {
 public:
  static bool save(const std::string&, const SavedMap&,
                   const MapPackageOptions&, std::string&);
  static bool load(const std::string&, SavedMap&,
                   const MapPackageOptions&, std::string&);
};
```

- [ ] **Step 5: Run focused tests and commit**

Run on Ubuntu:

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam_gtest_map_package_roundtrip_test
catkin_test_results
```

Expected: build succeeds and all map-package tests pass.

Commit: `git commit -m "feat: add versioned map package model"`

---

### Task 2: Stable BTC Serialization and Database Restore

**Files:**
- Create: `VoxelSLAM/src/btc_io.hpp`
- Create: `VoxelSLAM/src/btc_io.cpp`
- Create: `VoxelSLAM/test/btc_io_roundtrip_test.cpp`
- Modify: `VoxelSLAM/src/map_package.cpp`
- Modify: `VoxelSLAM/test/map_package_roundtrip_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `SavedBtcFrame`, `STD`, `BinaryDescriptor`, `STDescManager`.
- Produces: `save_btc_frames`, `load_btc_frames`, `restore_btc_database`.

- [ ] **Step 1: Write failing binary round-trip tests**

Test two frames, several STDs, unequal `occupy_array_` lengths, exact bit recovery, rewritten frame IDs, plane-cloud order, truncated files, wrong magic, huge declared lengths, and non-contiguous frame IDs.

```cpp
ASSERT_TRUE(save_btc_frames(path, input, error));
ASSERT_TRUE(load_btc_frames(path, output, error));
EXPECT_EQ(output[1].descriptors[0].binary_B_.occupy_array_,
          input[1].descriptors[0].binary_B_.occupy_array_);
```

- [ ] **Step 2: Run the test and observe the missing codec failure**

Run: `catkin_make run_tests_voxel_slam_gtest_btc_io_roundtrip_test`

Expected: compile failure for missing codec declarations.

- [ ] **Step 3: Implement the `VXLBTC01` codec**

Write exact-width integers, doubles, vectors, descriptor summaries/locations, and bit-packed arrays. Bound all counts before allocation. `restore_btc_database()` sorts by frame ID, verifies `0..N-1`, rewrites every `STD.frame_number_`, pushes the corresponding plane cloud, then calls `AddSTDescs()`.

```cpp
bool restore_btc_database(const std::vector<SavedBtcFrame>& frames,
                          STDescManager& manager,
                          std::string& error);
```

- [ ] **Step 4: Add search-equivalence coverage**

Construct a deterministic descriptor query and assert that candidate ID and transform score match before and after serialization/restoration. Explicitly set cross-session search so `skip_near_num_` cannot reject historical frames.

- [ ] **Step 5: Integrate BTC files into the map-package transaction**

`MapPackageIO::save()` writes `btc_frames.bin` and every `btc_planes/<frame_id>.pcd` before publishing the temporary directory. `MapPackageIO::load()` verifies manifest counts, loads planes, restores explicit anchor mappings, and fails the whole load on any missing or corrupt BTC artifact. Extend the map-package round-trip test to include two BTC frames.

- [ ] **Step 6: Run focused tests and commit**

Run on Ubuntu: `catkin_make run_tests_voxel_slam_gtest_btc_io_roundtrip_test && catkin_test_results`

Expected: codec, corruption, restore, and search-equivalence cases pass.

Commit: `git commit -m "feat: persist and restore BTC frames"`

---

### Task 3: Capture and Export Final Mapping Keyframes

**Files:**
- Modify: `VoxelSLAM/src/voxelslam.cpp`
- Modify: `VoxelSLAM/config/mid360.yaml`
- Create: `VoxelSLAM/test/mapping_capture_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `SavedMap`, `MapPackageIO`, existing fused `pptr`, `Keyframe::plptr`, `GenerateSTDescs()` output.
- Produces: in-memory one-to-one keyframe/BTC capture and final export.

- [ ] **Step 1: Extract and test a pure capture helper**

Test that accepted keyframe `k` records a local cloud copy, timestamp, anchor mapping, descriptor frame `k`, and copied `plane_cloud_vec_.back()`, while rejected motion creates no record.

```cpp
EXPECT_TRUE(captureAcceptedKeyframe(anchor, keyframe, descriptors, planes, saved, error));
EXPECT_EQ(saved.keyframes.back().id, saved.btc_frames.back().anchor_keyframe_id);
```

- [ ] **Step 2: Add mapping/localization mode configuration**

```yaml
General:
  mode: mapping
  map_export_path: ""
```

Reject unknown modes. Mapping mode requires a non-empty export path only when export is requested.

- [ ] **Step 3: Wire capture at the existing fused-keyframe boundary**

Immediately after `GenerateSTDescs()`, copy the accepted `Keyframe::plptr`, `stds_vec`, and the new plane cloud into `SavedMap`. Keep `frame_id` continuous and `anchor_keyframe_id` explicit.

- [ ] **Step 4: Export only after final optimization**

Before `MapPackageIO::save()`, replace every saved pose from the final optimized keyframe state. Any mismatch between saved anchors and optimized keyframes aborts export. Localization mode never calls this path.

- [ ] **Step 5: Run tests/static checks and commit**

Ubuntu: `catkin_make --pkg voxel_slam && catkin_make run_tests_voxel_slam_gtest_mapping_capture_test && catkin_test_results`

Windows checks:

```powershell
rg -n "mode|map_export_path|captureAcceptedKeyframe|MapPackageIO::save" VoxelSLAM
git diff --check
```

Commit: `git commit -m "feat: export optimized fused-keyframe maps"`

---

### Task 4: Load Immutable Maps and Build Spatial Indexes

**Files:**
- Create: `VoxelSLAM/src/prior_map_tracker.hpp`
- Create: `VoxelSLAM/src/prior_map_tracker.cpp`
- Create: `VoxelSLAM/test/prior_map_index_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: immutable `SavedMap`.
- Produces: `initialize`, `radius_search_xy`, `nearest_height_xy`, `update_loaded_keyframes`, `reset_runtime_cache`.

- [ ] **Step 1: Write failing XY/XYZ index tests**

Use non-contiguous insertion order and assert returned IDs come from explicit mappings, XY search ignores z, nearest height is selected by XY distance, and source-map hashes remain unchanged.

- [ ] **Step 2: Implement immutable indexes**

```cpp
class PriorMapTracker {
 public:
  bool initialize(const SavedMap&, std::string&);
  std::vector<int> radius_search_xy(const Eigen::Vector2d&, double) const;
  bool nearest_height_xy(const Eigen::Vector2d&, double&, int&, std::string&) const;
  void reset_runtime_cache();
};
```

Store XY points as `(x,y,0)` and maintain separate index-to-ID vectors for XY and XYZ trees.

- [ ] **Step 3: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_prior_map_index_test && catkin_test_results`

Expected: radius, height, mapping, and immutability tests pass.

Commit: `git commit -m "feat: index immutable historical keyframes"`

---

### Task 5: Filtered BTC Search and Expanding-Radius Relocalizer

**Files:**
- Modify: `VoxelSLAM/src/BTC.h`
- Modify: `VoxelSLAM/src/BTC.cpp`
- Create: `VoxelSLAM/src/relocalizer.hpp`
- Create: `VoxelSLAM/src/relocalizer.cpp`
- Create: `VoxelSLAM/test/relocalizer_transform_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: historical `STDescManager`, keyframe-to-BTC mappings, `PriorMapTracker`, current descriptors/plane cloud.
- Produces: global or filtered `RelocalizationResult` containing history anchor, `T_H_C`, score, ICP observability, and failure reason.

- [ ] **Step 1: Add failing candidate-filter tests**

Assert out-of-set descriptors contribute no votes, an empty allowed set returns no candidate, the global overload is unchanged, and a high-vote out-of-range frame cannot hide the correct allowed frame.

- [ ] **Step 2: Add filtered overloads**

```cpp
void SearchLoop(/* existing args */,
                const std::unordered_set<int>& allowed_frame_ids);
void candidate_selector(std::vector<STD>&,
                        std::vector<STDMatchList>&,
                        const std::unordered_set<int>* allowed_frame_ids,
                        bool cross_session);
```

Filter before incrementing `match_array`; when `cross_session` is true, do not apply same-session `skip_near_num_` rejection.

- [ ] **Step 3: Test and implement radius policy**

For initial/max radii 30/100, assert attempts are exactly 30, 60, 100, then global only if enabled.

```cpp
double next_radius(double current, double maximum) {
  return std::min(current * 2.0, maximum);
}
```

- [ ] **Step 4: Confirm transform direction with synthetic geometry**

Generate asymmetric 3D planes, apply known 90-degree yaw and nonzero XYZ translation, then assert output obeys `p_H = R_HC*p_C+t_HC`. Include a geometrically degenerate case that must fail `icp_normal()`.

- [ ] **Step 5: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_relocalizer_transform_test && catkin_test_results`

Commit: `git commit -m "feat: add bounded and global BTC relocalization"`

---

### Task 6: Fixed-Anchor GTSAM Session Alignment

**Files:**
- Create: `VoxelSLAM/src/session_aligner.hpp`
- Create: `VoxelSLAM/src/session_aligner.cpp`
- Create: `VoxelSLAM/test/session_aligner_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: historical anchor pose, current anchor index, `T_H_C`, current odometry poses.
- Produces: optimized current poses and `T_W_O`.

- [ ] **Step 1: Write the five-node drift test**

Assert the history anchor remains fixed, current nodes enter map coordinates, relative odometry structure is preserved, NaN and unsafe jumps fail, and deliberately inverted `T_H_C` fails the expected-pose assertion.

- [ ] **Step 2: Implement the small alignment graph**

```cpp
bool align_current_session_to_map(
    const SavedPose& history_anchor,
    int current_anchor_index,
    const Eigen::Matrix3d& R_HC,
    const Eigen::Vector3d& t_HC,
    const std::vector<ScanPose*>& current_poses,
    std::vector<gtsam::Pose3>& optimized,
    IMUST& T_W_O,
    std::string& error);
```

Use a strong prior only on history anchor H, odometry between-factors for current nodes, and one H-to-C cross-session factor. Do not retain the temporary odom-origin prior.

- [ ] **Step 3: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_session_aligner_test && catkin_test_results`

Commit: `git commit -m "feat: align localization sessions to map"`

---

### Task 7: Atomic Frontend Global Correction

**Files:**
- Create: `VoxelSLAM/src/global_correction.hpp`
- Create: `VoxelSLAM/src/global_correction.cpp`
- Create: `VoxelSLAM/test/global_correction_test.cpp`
- Modify: `VoxelSLAM/src/voxelslam.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `T_W_O`, current state, window states, cached poses/clouds, correction generation ID.
- Produces: corrected copies and one-time commit through `VOXEL_SLAM::apply_global_correction`.

- [ ] **Step 1: Write pure correction tests**

Check pose, velocity, gravity, covariance rotation, cached trajectory, and local points under a known rigid transform. Applying the same generation twice must be rejected.

- [ ] **Step 2: Implement pure helpers and commit barrier**

```cpp
bool apply_global_correction_once(
    const IMUST& T_W_O, uint64_t generation,
    CorrectionTargets& targets, std::string& error);
```

Rotate gravity only, transform velocity only by rotation, and transform covariance with the error-state adjoint blocks used by `IMUST`.

- [ ] **Step 3: Refactor `loop_update()` to reuse the helper**

Keep existing loop behavior but route pose/state correction through the tested implementation. Hold the existing loop/state mutex across validation and commit so readers never see a partially corrected window.

- [ ] **Step 4: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_global_correction_test && catkin_test_results`

Commit: `git commit -m "refactor: apply global corrections atomically"`

---

### Task 8: Read-Only Historical Local Map and IESKF Tracking

**Files:**
- Modify: `VoxelSLAM/src/prior_map_tracker.hpp`
- Modify: `VoxelSLAM/src/prior_map_tracker.cpp`
- Create: `VoxelSLAM/test/prior_map_loading_test.cpp`
- Modify: `VoxelSLAM/src/voxelslam.cpp`

**Interfaces:**
- Consumes: corrected map-frame state and immutable keyframes.
- Produces: fixed historical `surf_map`, loaded-ID cache, `TrackingQuality` for every scan.

- [ ] **Step 1: Write loading/unloading and immutability tests**

Simulate motion through three separated keyframe groups. Assert radius membership, at most one new load per scan, no duplicate loads, unload behavior, and identical `SavedMap` hash before/after.

- [ ] **Step 2: Implement local-map cache lifecycle**

```cpp
bool rebuild_local_map(const Eigen::Vector3d&, double radius, int preload_limit);
bool update_loaded_keyframes(const Eigen::Vector3d&, double load_radius,
                             double unload_radius, int max_load_per_scan);
```

Transform historical local clouds with final `T_W_K`, write only fixed historical points, complete recut/plane fitting before tracking, and release only cache-owned historical voxel objects.

- [ ] **Step 3: Add localization-only LIO branch**

In `IN_MAP_TRACKING`, run IMU propagation, undistortion, and point-to-plane IESKF update against the fixed historical layer. Skip `cut_voxel_multi()` for current scans; any short-lived current cloud lives in a separate cache cleared on state exit.

- [ ] **Step 4: Emit quality metrics**

```cpp
struct TrackingQuality {
  int matched_points;
  double mean_abs_point_plane;
  double min_normal_eigenvalue;
  bool finite;
  bool solve_succeeded;
};
```

- [ ] **Step 5: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_prior_map_loading_test && catkin_test_results`

Commit: `git commit -m "feat: track against read-only historical maps"`

---

### Task 9: RViz Direct Initialization

**Files:**
- Create: `VoxelSLAM/src/localization_state.hpp`
- Create: `VoxelSLAM/test/direct_pose_initializer_test.cpp`
- Modify: `VoxelSLAM/src/prior_map_tracker.hpp`
- Modify: `VoxelSLAM/src/prior_map_tracker.cpp`
- Modify: `VoxelSLAM/src/voxelslam.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `/initialpose`, IMU roll/pitch, nearest historical height, current odom anchor.
- Produces: versioned direct-pose request, refined candidate `T_W_O`, validation streak.

- [ ] **Step 1: Test seed construction and request replacement**

Assert x/y/yaw come from RViz, roll/pitch from IMU, z from nearest XY keyframe, non-map frames fail, IMU-not-ready fails, and request N+1 invalidates result N.

- [ ] **Step 2: Add callback-only request capture**

```cpp
struct PendingInitialPose {
  uint64_t generation = 0;
  ros::Time stamp;
  Eigen::Vector2d xy;
  double yaw = 0.0;
};
```

The callback validates and stores only. Heavy map building, refinement, and frontend changes run in the localization thread.

- [ ] **Step 3: Implement direct validation flow**

Build the historical local map, refine the fused current cloud on copies, calculate `T_W_O`, atomically correct the frontend, and keep `localization_valid=false` until `direct_pose_validation_scans` consecutive scans pass. Timeout or hard failure resets temporary LIO and resumes configured BTC. A newer RViz request cancels the old request and starts immediately without BTC fallback between them.

- [ ] **Step 4: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_direct_pose_initializer_test && catkin_test_results`

Commit: `git commit -m "feat: validate RViz map initial poses"`

---

### Task 10: Localization State Machine, LOST, and ROS Status

**Files:**
- Modify: `VoxelSLAM/src/localization_state.hpp`
- Create: `VoxelSLAM/test/localization_state_test.cpp`
- Modify: `VoxelSLAM/src/voxelslam.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: map-load result, relocalization result, alignment result, `TrackingQuality`, RViz requests.
- Produces: legal transitions, cleanup actions, `std_msgs/String` state and `std_msgs/Bool` valid topics.

- [ ] **Step 1: Write transition-table tests**

Cover map load, BTC success/failure, direct success/timeout, GOOD→five BAD→LOST, one-frame NaN→LOST, cleanup count, and LOST→LOCAL_ODOMETRY while retaining immutable map handles.

- [ ] **Step 2: Implement explicit reducer-style transitions**

```cpp
enum class LocalizationState {
  WAITING_FOR_MAP, LOCAL_ODOMETRY, RELOCALIZING,
  ALIGNING, IN_MAP_TRACKING, LOST
};
enum class AlignmentSource { BTC, RVIZ_INITIAL_POSE };
```

Reject illegal events with a warning and no state change. Hard failures transition immediately; soft failures increment a streak reset by any good frame.

- [ ] **Step 3: Wire cleanup and status publication**

LOST clears current-session windows, query descriptors, temporary clouds, and validity, but retains `SavedMap`, historical BTC database, and indexes. Return to temporary LIO and configured BTC automatically.

- [ ] **Step 4: Run tests and commit**

Ubuntu: `catkin_make run_tests_voxel_slam_gtest_localization_state_test && catkin_test_results`

Commit: `git commit -m "feat: recover localization after tracking loss"`

---

### Task 11: Configuration, Launch, Evaluation, and End-to-End Audit

**Files:**
- Create: `VoxelSLAM/config/localization_mid360.yaml`
- Create: `VoxelSLAM/launch/vxlm_localization_mid360.launch`
- Create: `VoxelSLAM/scripts/evaluate_localization.py`
- Modify: `VoxelSLAM/config/mid360.yaml`
- Modify: `VoxelSLAM/README.md`

**Interfaces:**
- Consumes: all completed runtime features.
- Produces: reproducible mapping/localization commands, ATE/RPE report, map hash report.

- [ ] **Step 1: Add complete documented defaults**

Include map path, local-map load/unload radii, load limits, quality thresholds, LOST streak, bounded-BTC keys, RViz validation keys, and alignment safety limits. Document which fields are ignored when `use_prior_pose_for_btc=false`.

- [ ] **Step 2: Add launch and offline evaluator**

The launch file loads localization YAML and starts the same executable in localization mode. The evaluator accepts TUM-format estimate/reference files plus a map directory and prints ATE RMSE, translational/rotational RPE, and SHA-256 hashes.

- [ ] **Step 3: Run Windows-available checks**

```powershell
git diff --check
rg -n "BCT" VoxelSLAM/src VoxelSLAM/config VoxelSLAM/launch
rg -n "use_prior_pose_for_btc|btc_search_radius|rviz_initialpose_enabled" VoxelSLAM
python -m py_compile VoxelSLAM/scripts/evaluate_localization.py
```

Expected: no whitespace errors, no `BCT` spelling, all configuration keys are consumed/documented, Python compiles.

- [ ] **Step 4: Run the full Ubuntu verification gate**

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
```

Expected: build exit 0 and zero failed tests.

- [ ] **Step 5: Run end-to-end datasets**

1. Map once and export a package after optimization.
2. Relocalize from a different position/yaw with global BTC.
3. Repeat with bounded BTC and verify every selected history anchor lies inside the active XY radius.
4. Send a correct RViz pose and confirm validity only after the configured streak.
5. Send an incorrect RViz pose and confirm automatic BTC fallback.
6. Replay non-overlap data and confirm no false valid state.
7. Force geometric loss, confirm LOST, and re-enter known geometry for recovery.
8. Compare map-directory SHA-256 before/after; every hash must match.

- [ ] **Step 6: Commit final integration**

Commit: `git commit -m "docs: add map localization workflow and evaluation"`

---

## Completion Evidence

Do not call the implementation complete until all of the following exist:

- Git log shows each task commit and `git status --short` is empty.
- Ubuntu `catkin_make --pkg voxel_slam` exits 0.
- Ubuntu `catkin_test_results` reports zero failures.
- Synthetic BTC/ICP test confirms `T_H_C` direction.
- Global BTC, bounded BTC, and RViz direct initialization each reach valid tracking on overlapping data.
- Non-overlap data never reaches valid tracking.
- LOST recovery succeeds on a replay that re-enters mapped space.
- Historical map hashes are identical before and after localization.

On this Windows host, report only the tasks actually established by source inspection and available pure-script checks; list every Ubuntu gate as unverified until its logs are supplied.
