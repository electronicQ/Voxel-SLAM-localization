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
