# ROS 2 Humble 运行说明

`ros2-humble` 分支保留原有地图包和定位算法，运行时改为 ROS 2 Humble、`ament_cmake`、`colcon`、`tf2` 和 `rosbag2`。

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- PCL、Eigen、Boost、GTSAM
- ROS 2 Livox 驱动（`livox_ros_driver2`）
- `rviz2` 和 `rosbag2_storage_mcap`

安装驱动后，在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

若工作空间中同时有 `third_party/install/gtsam` 和 `install/gtsam`，必须
保持编译和运行所用的 GTSAM 一致。即使都是 4.2，不同的 TBB 分配器或
Eigen 配置也可能造成 `NonlinearFactor::rekey` 等符号无法解析。本项目在
Linux 下使用 RPATH 固定 CMake 选中的 GTSAM 库，防止被另一份同名库覆盖。
更换 GTSAM 时应同步更新 `GTSAM_DIR` 和缓存的 `GTSAM_INCLUDE_DIR`，然后重新构建。

没有 Livox 驱动时，节点仍可订阅标准 `sensor_msgs/msg/PointCloud2`，但不会解析 Livox 自定义点时间字段。

## 建图

```bash
ros2 launch voxel_slam vxlm_mid360.launch.py rviz:=true
```

通过参数文件中的 `General.map_export_path` 配置地图导出目录。定位模式使用同一版本生成的 v2 地图包。

## 定位

```bash
ros2 launch voxel_slam vxlm_localization_mid360.launch.py \
  map_path:=/absolute/path/to/v2_map rviz:=true
```

检查定位状态和高频里程计：

```bash
ros2 topic echo /localization/state
ros2 topic echo /localization/valid
ros2 topic hz /LIO/odom_imu
```

Avia 飞行数据及其对应地图使用专用启动文件，保留建图时的 Avia 外参和
高空 BTC 描述子设置：

```bash
ros2 launch voxel_slam vxlm_localization_avia_fly.launch.py \
  map_path:=/absolute/path/to/HKisland03_localization rviz:=true
```

该配置的空间块核心/加载/卸载半径为 100/120/150 米。HKisland03 的飞行
轨迹到最近地图块的距离可达约 86.5 米，因此不适合直接使用 Mid360
配置中的 15/30/40 米范围。地图中的融合网格和根体素参数仍由地图包读取。

RViz2 的 `SetInitialPose` 工具发布 `/initialpose`，固定坐标系为 `camera_init`。

## rosbag2

使用 MCAP 回放：

```bash
ros2 bag play /absolute/path/to/session --storage-id mcap --clock
```

回放前确认 Livox 驱动或录制数据中的话题名称与 `General/lid_topic`、`General/imu_topic` 一致。

## 导航适配器

```bash
ros2 launch voxel_slam scan_planner_adapter.launch.py
```

适配器订阅 `/LIO/odom_imu` 和 `/localization/valid`，读取 `aft_mapped -> base_link` TF 后发布 `/LIO/odom_vehicle`。第三方 SCAN-Planner 必须使用 ROS 2 版本，或通过 `ros1_bridge` 接入。

## 验证

```bash
colcon test --packages-select voxel_slam --event-handlers console_direct+
colcon test-result --verbose
```

完整运行验证还需要在目标 Livox 数据和实际 v2 地图包上执行建图、回放、RViz2 初始位姿和定位丢失恢复测试。
