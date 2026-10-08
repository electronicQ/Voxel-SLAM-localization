# Voxel-SLAM ROS1 功能包说明

本目录是仓库中的 ROS1 `voxel_slam` 功能包。完整建图、v2 空间块地图、BTC/RViz 重定位和评测说明见仓库根目录 `README.md`；本文重点说明高频导航接口。

## 高频状态输出

系统同时保留两条 IMU 数据路径：

1. 正式路径把 IMU 缓存在 `imu_buf`，与一帧激光雷达同步后执行去畸变、协方差传播和 IESKF 点到面更新。该路径产生权威后验。
2. 快速路径在独立单线程 callback queue 中，从最近后验按每条新 IMU 传播状态均值，发布 `/LIO/odom_imu` 和 `camera_init -> aft_mapped`。它不修改正式滤波器，不重复使用快速预测作为下一帧 IESKF 初值。

快速预测默认最多领先最近后验 0.5 秒；超时、IMU 时间回退、系统重置或定位丢失时停止输出并等待新后验。定位模式默认还要求 `/localization/valid=true`。

## 车辆里程计适配

`scan_planner_odom_adapter` 订阅：

- `/LIO/odom_imu`
- `/localization/valid`
- TF 中的静态 `aft_mapped -> base_link`

并发布 `/LIO/odom_vehicle`。适配器按完整刚体变换组合机体位姿，并计算角速度引起的杆臂线速度；它不发布任何 TF。机器人安装外参必须由用户的 URDF、`robot_state_publisher` 或静态 TF 发布器提供。

默认配置位于 `config/scan_planner_adapter.yaml`，启动命令为：

```bash
roslaunch voxel_slam scan_planner_adapter.launch
```

TF 树应为：

```text
camera_init -> aft_mapped -> base_link
```

不要再发布 `base_link -> aft_mapped`，否则 `aft_mapped` 会同时具有动态和静态父节点。

## ROS1 SCAN-Planner 参数

[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 是外部工程，不随本仓库 vendored。可以放入同一个 catkin 工作空间一起编译，也可以单独编译后依次 source Voxel-SLAM 与 SCAN-Planner overlay；完整接入和 launch 裁剪流程见仓库根目录 `README.md`。

```text
body_pose_topic=/LIO/odom_vehicle
sensor_pose_topic=/LIO/odom_imu
cloud_topic=/map_scan
grid_map/frame_id=camera_init
grid_map/cloud_is_world=true
grid_map/need_extrinsic=true
```

`cloud_is_world` 与 `need_extrinsic` 语义独立：`/map_scan` 已在世界系，因此保持 `cloud_is_world=true`；`sensor_pose_topic=/LIO/odom_imu` 提供的是 IMU 位姿，因此保持 `need_extrinsic=true`，由 LiDAR→IMU 外参得到射线原点。默认 Mid-360 平移为 `[-0.011, -0.02329, 0.04412]`、旋转为单位旋转，必须与 `config/localization_mid360.yaml` 中的 `General/extrinsic_tran` 和 `General/extrinsic_rota` 一致。

上游 `run.launch` 会无条件设置 Go2 `robot_description` 并启动 `go2_robot_state_publisher`。使用自有 URDF 时应复制并裁剪该 launch，避免重复启动 `robot_state_publisher`、覆盖机器人描述或发布冲突 TF。

## Ubuntu 编译与验证

```bash
cd ~/catkin_ws
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results

rostopic hz /LIO/odom_imu
rostopic hz /LIO/odom_vehicle
rostopic delay /LIO/odom_imu
rosrun tf tf_echo camera_init aft_mapped
rosrun tf tf_echo aft_mapped base_link
```

当前 Windows 修改环境只能执行 Python 源码契约和配置检查，不能代替以上 ROS/C++ 验证。
