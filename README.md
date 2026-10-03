# Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System

## 1. Introduction

**Voxel-SLAM** is a complete, accurate, and versatile LiDAR-inertial SLAM system that fully utilizes short-term, mid-term, long-term, and multi-map data associations. It includes five modules: initialization, odometry, local mapping, loop closure, and global mapping. The initialization can provide accurate states and local map in a static or dynamic initial state. The odometry estimates current states and detect potential system divergence. The local mapping refine the states and local map within the sliding window by a LiDAR-inertial BA. The loop closure can detect in multiple sessions. The global mapping refine the global map with an efficient hierarchical global BA. The system overview is:

<div align="center">
    <a href="https://youtu.be/Cg9W01aIUzE" target="_blank">
    <img src="./figure/systemoverview.png" width = 60% >
</div>

### 1.1 Related Video

The video of **Voxel-SLAM** is available on [YouTube](https://youtu.be/Cg9W01aIUzE).

### 1.2 Related works

Related paper is available on [**arxiv**](https://arxiv.org/abs/2410.08935).

### 1.3 Competitions

Voxel-SLAM has been served as a subsystem to participate in [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html) (**2nd** place on the LiDAR single-session) and [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI) (**1st** place on the LiDAR inertial track).

## 2. Prerequisited

Ubuntu=20.04. [ROS =Noetic](http://wiki.ros.org/ROS/Installation). [PCL=1.10](https://pointclouds.org/). [Eigen=3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page)

[GTSAM=4.0.3](https://github.com/borglab/gtsam/releases?page=2)

[livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)

## 3. Build

```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/Voxel-SLAM
cd ../ && catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run Voxel-SLAM

### 4.1 Livox Avia

The online relocalization experiment rosbag. Download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErEznhkJzTxJiLuJ8AQDGS0BvCy6KsuaWF2D6cnx061GEQ?e=dMRSlf) ([Google Drive](https://drive.google.com/file/d/1LG46i0vreQrMZRap5tJkjKK4IX0t0zfC/view?usp=drive_link))

```
roslaunch roslaunch voxel_slam vxlm_avia.launch
// Using the "--pause" guarantees the bag benning time are the same in different runs
// Press the Space to start
rosbag play compus_elevator.bag --pause 
```

In the elevator, the system continues to restart until stepping out of the evevator. The blue point cloud is the map from initialization.

After the rosbag is done, your may find the map is inconsistent as shown in the video. Run

```
rosparam set finish true
```

to launch the final global mapping (global bundle adjustment) to refine the global map.

### 4.2 HILTI 2023 (Multi-Session)

The multi-session experiment rosbag. 

For quick test to download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/Epp5AQ2Oq1VNhC6MIuCAtN4BJC9jx9VvuVx7VT_cdlvD0A?e=8mXYdc). The whole rosbags of HILTI 2022 and 2023 are on the [website](https://hilti-challenge.com/index.html).

The rosbag had better be played from "site1_handheld_5" to "site_handheld_1", or the "site_handheld_2" and "site_handheld_3" cannot find the loop. 

Before launching, please set configure the variables in "hesai.yaml". The '#' means annotation

```
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "# site1_handheld_5: 0.50, 
               # site1_handheld_4: 0.45,
               # site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_${1-5}" # The rosbag name you play
is_save_map: 1 # Enable to save the map
```

```
roslaunch voxel_slam vxlm_hesai.launch
rosbag play site1_handheld_5.bag --pause
```

```
roslaunch voxel_slam vxlm_hesai.launch // Load the site_handheld_5
rosbag play site1_handheld_4.bag --pause
```

```
roslaunch voxel_slam vxlm_hesai.launch // Load the site_handheld_{5, 4}
rosbag play site1_handheld_3.bag --pause
```

For the "site1_handheld_2", do not forget load the offline maps. The "hesai.yaml" should be like this

```
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "site1_handheld_5: 0.50, 
               site1_handheld_4: 0.45,
               site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_2" # The rosbag name you play
is_save_map: 1 # Enable to save the map
```

```
roslaunch voxel_slam vxlm_hesai.launch // Load the site_handheld_{5, 4, 3}
rosbag play site1_handheld_2.bag --pause
```

```
roslaunch voxel_slam vxlm_hesai.launch // Load the site_handheld_{5, 4, 3, 2}
rosbag play site1_handheld_1.bag --pause
```

The map may not be consistent as shown in the video. Run

```
rosparam set finish true
```

for the final global BA.

### 4.3 MARS Dataset

For quick test to download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/EpjsGW6coYlMvBWo8TlgJXoBttAuoocLi24V6kw-r_3A8w?e=vVB6RY). The whole rosbags of MARS dataset are on the [website](https://mars.hku.hk/dataset.html).

```
roslaunch voxel_slam vxlm_avia_fly.launch
rosbag play HKisland03.bag --pause
```

The beginning of the point cloud is empty and failing to initialize until the drone at a certain height.

```
roslaunch voxel_slam vxlm_avia_fly.launch
rosbag play AMvalley03.bag --pause
rosparam set finish true
```

This sequence is difficult to find loop. Please run the GBA to ensure the global map consistence.

### 4.4 Livox Mid360

The rosbag begin in a violent speed: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErtuXCFhFrBErZxzS5vLASkBJEfgDB9R2CSCgKe8BwhneQ?e=zbr7NL)

```
roslaunch voxel_slam vxlm_mid360.launch
rosbag play jungle_challenge.bag --pause
```

### 4.5 Build a reusable map and localize in it

Mapping and localization use the same `voxelslam` executable. To export a map
package, keep `General/mode: mapping` and set `General/map_export_path` in
`VoxelSLAM/config/mid360.yaml`. The destination must not exist yet, while its
parent directory must already exist. Run mapping normally, then request final
global optimization and export:

```bash
roslaunch voxel_slam vxlm_mid360.launch
rosbag play mapping.bag --pause
rosparam set finish true
```

The package contains optimized keyframe poses, local-frame keyframe clouds,
BTC descriptors and BTC plane clouds. It is written transactionally only after
the final optimization completes.

For localization, set `Localization/map_path` in
`VoxelSLAM/config/localization_mid360.yaml`, or pass it to the launch file:

```bash
roslaunch voxel_slam vxlm_localization_mid360.launch \
  map_path:=/absolute/path/to/map_package
rosbag play localization.bag --pause
```

`use_prior_pose_for_btc: true` restricts BTC history candidates to keyframes
within `btc_search_radius` of `btc_prior_pose.x/y`. Failures expand the radius
up to `btc_max_search_radius`, then optionally search globally. Candidate
selection deliberately has no yaw constraint. If `use_prior_pose_for_btc` is
false, the prior pose and all radius/expansion/fallback settings are ignored
and BTC starts with a global search.

RViz **2D Pose Estimate** is always an independent initialization path when
`rviz_initialpose_enabled` is true. Its x/y/yaw are treated as the current
vehicle pose in `map`; roll/pitch come from the live IMU and z comes from the
nearest historical keyframe in XY. It bypasses BTC, but localization becomes
valid only after `direct_pose_validation_scans` consecutive local historical
map matches. A newer RViz request replaces an older request.

The latched status topics are:

- `/localization/state` (`std_msgs/String`): `WAITING_FOR_MAP`,
  `LOCAL_ODOMETRY`, `RELOCALIZING`, `ALIGNING`, `IN_MAP_TRACKING`, or `LOST`.
- `/localization/valid` (`std_msgs/Bool`): true only after BTC alignment or RViz
  local-map validation has succeeded.

After the configured number of bad tracking scans, or immediately after a
non-finite solve, the system enters `LOST`, clears only current-session state,
and automatically returns to local odometry plus BTC recovery. The loaded map,
historical BTC database, and keyframe indexes remain immutable.

To evaluate a TUM-format estimate against a map-frame TUM reference and verify
that the map package did not change:

```bash
python3 VoxelSLAM/scripts/evaluate_localization.py \
  --estimate estimate.tum \
  --reference reference.tum \
  --map-dir /absolute/path/to/map_package
```

The evaluator reports map-frame ATE RMSE without post-hoc alignment,
translational/rotational RPE, per-file SHA-256 values, and a deterministic
whole-tree SHA-256. Re-run it before and after localization; all map hashes must
remain identical.

### 4.6 Others

Other types of LiDAR will be released later.

## 5. VoxelSLAMPointCloud2

**VoxelSLAMPointCloud2**: A customized plugin for RViz. It has the same usage to original "PointCloud2" in RViz, but it can **clear the point cloud map automatically** when receiving an empty point cloud, with any **Decay Time** of the plugin. 

(1) Put the "VoxelSLAMPointCloud2" within the same "src" folder to your project and catkin_make.

(2) “roslaunch” your program with RViz. 

(3) Click the "Add" button to add the "VoxelSLAMPointCloud2".

