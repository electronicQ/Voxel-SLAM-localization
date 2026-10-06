# Voxel-SLAM高频里程计与ROS1 SCAN-Planner适配设计

## 1. 目标

在现有Voxel-SLAM正式LiDAR-IMU IESKF链之外增加一条高频IMU预测输出链，并为ROS1 SCAN-Planner提供下列接口：

- `/LIO/odom_imu`：IMU频率的传感器位姿；
- TF `camera_init -> aft_mapped`：与高频传感器里程计一致；
- `/LIO/odom_vehicle`：由独立适配节点结合URDF静态外参生成；
- `/map_scan`：保留现有激光频率的世界系去畸变点云。

改造不得改变正式IESKF的批量IMU传播、点云去畸变和激光观测更新流程，也不得把机器人安装外参写死在Voxel-SLAM中。

## 2. 当前事实与边界

- Voxel-SLAM是ROS1 Noetic工程。
- 正式前端通过`sync_packages()`收集覆盖一帧激光时间范围的IMU，再由`IMUEKF::process()`完成状态和协方差传播及点云去畸变。
- 当前动态TF只在激光帧处理后由`ResultOutput::pub_odom_func()`发布，父子坐标系为`camera_init -> aft_mapped`，时间戳错误地使用发布时刻而不是状态时刻。
- 当前IMU订阅依赖主线程中的`ros::spinOnce()`；滑窗BA、地图加载或其他长计算会阻塞IMU回调。
- 重定位成功后的`IN_MAP_TRACKING`使用不可变历史空间块进行逐扫描IESKF更新，并通过`continue`跳过滑窗BA和当前扫描入图。
- 重定位成功前的临时`LOCAL_ODOMETRY`继续保留现有滑窗BA，以稳定临时轨迹、融合关键帧和BTC查询帧。
- 原版ROS1 SCAN-Planner真机接口使用`/LIO/odom_vehicle`、`/LIO/odom_imu`和`/LIO/clouds_lidar`。本适配不伪造传感器系点云，而是把SCAN配置为直接消费世界系`/map_scan`。

## 3. 总体架构

```text
专用IMU回调线程
  ├── 原有imu_buf
  │     └── 激光帧到齐后供正式IESKF批量处理
  │
  └── ImuRatePredictor
        ├── 最近一次正式后验快照
        ├── 快照之后的IMU尾部
        ├── 只传播R/p/v状态均值
        └── 发布/LIO/odom_imu与高频TF

Voxel-SLAM /LIO/odom_imu
        +
URDF静态TF（IMU到base_link）
        ↓
scan_planner_odom_adapter
        ↓
/LIO/odom_vehicle
```

正式IESKF和高频预测器使用同一批原始IMU，但作用于相互独立的状态副本：正式链负责协方差、去畸变和激光更新；高频链只负责向控制器提供接近当前IMU时刻的暂态预测。不存在在同一正式状态上重复积分的问题。

## 4. 高频预测器

### 4.1 职责

新增独立、可单元测试的`ImuRatePredictor`，提供以下行为：

```cpp
void setEnabled(bool enabled);
void updateSnapshot(const IMUST &posterior, double acceleration_scale);
PredictionResult pushImu(const ImuSample &sample);
void reset();
```

`PredictionResult`至少包含：

- 是否允许发布；
- 预测状态`IMUST`；
- 最新去零偏角速度；
- 拒绝原因（未初始化、时间非递增、预测跨度过长、状态非有限等）。

### 4.2 传播语义

后验快照包含`R/p/v/bg/ba/g/t`。收到新IMU后按以下方程传播：

```text
omega = gyro - bg
acc_body = acceleration_scale * accel - ba
acc_world = R * acc_body + g
p_next = p + v * dt + 0.5 * acc_world * dt^2
v_next = v + acc_world * dt
R_next = R * Exp(omega, dt)
```

高频链不更新正式协方差。正常情况下每帧新IMU只做一次增量传播；当新的后验快照替换旧快照时，预测器从新后验重放快照之后仍然有效的IMU尾部一次，以吸收激光、BA或全局对齐修正。

### 4.3 数据有效性

- 首个正式后验产生前不发布。
- IMU时间戳必须严格递增；时间回跳时清空尾部并等待新后验。
- 预测状态、输入IMU和时间间隔必须全部有限。
- `latest_imu_time - posterior_time`超过`max_prediction_sec`时停止发布并限频报警，禁止无限纯IMU外推。
- 消息时间戳使用最新IMU的传感器时间，禁止使用`ros::Time::now()`代替测量时间。

## 5. ROS并发模型

新增专用`ros::CallbackQueue`和单线程`ros::AsyncSpinner`处理IMU订阅。激光订阅和主前端继续使用原有主回调队列。

IMU回调执行顺序：

1. 复制和校验IMU消息；
2. 在原有`mBuf`保护下写入`imu_buf`并更新`imu_last_time`；
3. 将同一测量交给`ImuRatePredictor`；
4. 若预测有效且满足发布门控，则发布里程计与TF。

预测器内部使用独立互斥量保护后验、尾部IMU和传播状态。发布前复制结果并释放互斥量，避免ROS发布阻塞正式前端更新快照。

节点析构或主流程结束时必须先停止IMU spinner，再销毁预测器和ROS发布器。

## 6. 快照刷新与状态机

### 6.1 建图模式

- 首个有效IESKF后验产生后允许发布高频里程计。
- 每次激光IESKF成功后刷新快照。
- 临时滑窗BA修改当前状态后再次刷新快照。
- 回环或全局校正应用后，用校正后的状态替换快照并重放尾部IMU。

### 6.2 定位模式：重定位前

- `LOCAL_ODOMETRY`保留现有滑窗BA和BTC查询帧构造。
- 默认`publish_before_localized=false`，此阶段不向SCAN-Planner发布导航里程计。
- 内部预测器仍可维护快照，以便诊断和在定位转为有效时无缝接管。

### 6.3 定位模式：重定位后

- BTC或RViz位姿通过局部历史地图验证并完成全局校正后，立即以校正状态刷新快照。
- `PREPARING_LOCAL_MAP`和`IN_MAP_TRACKING`不运行滑窗BA。
- 每次历史地图点到面IESKF更新成功后刷新快照。
- 仅当`/localization/valid=true`时向SCAN发布。

### 6.4 LOST与重置

以下事件必须调用`reset()`并停止有效输出：

- 定位进入LOST；
- `system_reset()`；
- IMU时间回跳；
- 正式状态或预测状态出现NaN/Inf；
- 前端切换到新的临时session。

## 7. Voxel-SLAM ROS接口

新增配置：

```yaml
Odometry:
  imu_rate_output: true
  imu_odom_topic: "/LIO/odom_imu"
  odom_frame: "camera_init"
  imu_child_frame: "aft_mapped"
  publish_before_localized: false
  max_prediction_sec: 0.5
```

`/LIO/odom_imu`消息约定：

- `header.stamp`：最新IMU传感器时间；
- `header.frame_id`：配置的`odom_frame`；
- `child_frame_id`：配置的`imu_child_frame`；
- `pose`：预测的`T_odom_imu`；
- `twist.linear`：世界/里程计坐标系速度，保持与ROS1 SCAN-Planner现有读取方式一致；
- `twist.angular`：最新IMU角速度减去后验陀螺零偏，表达在IMU坐标系；
- `pose.covariance`：从最近一次正式后验复制，并明确它不代表高频预测后的完整协方差；SCAN适配不依赖该字段。

动态TF与里程计使用相同位姿和时间戳。启用高频输出并已有后验后，原激光频率TF停止发布，避免两个发布源竞争同一父子TF；关闭高频输出时保留原扫描率TF作为兼容回退。

## 8. 车辆里程计适配节点

新增`scan_planner_odom_adapter`，职责仅限导航接口转换，不参与定位估计。

参数：

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

适配节点从URDF/`robot_state_publisher`获取静态`T_imu_base`，计算：

```text
T_odom_base = T_odom_imu * T_imu_base
```

车辆中心世界系线速度按杠杆臂修正：

```text
v_odom_base = v_odom_imu + omega_odom × (R_odom_imu * t_imu_base)
```

其中`omega_odom = R_odom_imu * omega_imu`。输出`/LIO/odom_vehicle`的线速度继续采用SCAN-Planner当前所需的世界系语义。

适配节点不发布新的动态TF，避免与用户URDF或其他机器人状态节点形成多父节点。TF缺失、定位无效、输入超时或状态非有限时不发布，并使用限频日志说明原因。

## 9. SCAN-Planner接线

ROS1 SCAN-Planner按下列接口配置：

```text
body_pose_topic   = /LIO/odom_vehicle
sensor_pose_topic = /LIO/odom_imu
cloud_topic       = /map_scan
grid_map/frame_id = camera_init
cloud_is_world    = true
need_extrinsic    = false
```

`/map_scan`已经位于`camera_init`世界系，因此SCAN不得再次使用传感器位姿变换该点云。系统不发布一个名为`/LIO/clouds_lidar`但实际位于世界系的兼容别名，以避免坐标系语义错误。

## 10. 测试与验证

### 10.1 纯逻辑单元测试

- 静止、重力平衡IMU传播后位置和速度保持稳定；
- 已知初速度下位置按匀速模型变化；
- 恒定角速度产生正确yaw；
- 新后验替换后正确重放后验之后的IMU尾部；
- 全局位姿校正后不再从旧坐标系状态继续传播；
- 时间非递增、NaN输入和预测跨度超限时拒绝发布；
- `reset()`后必须等待新后验；
- 已知`T_imu_base`下车辆位姿和杠杆臂速度计算正确。

### 10.2 ROS集成验证

Ubuntu 20.04 / ROS Noetic目标机执行：

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
rosbag play <bag>
rostopic hz /LIO/odom_imu
rostopic hz /LIO/odom_vehicle
rostopic hz /map_scan
rostopic delay /LIO/odom_imu
rosrun tf tf_echo camera_init aft_mapped
```

验证滑窗BA或空间块加载期间`/LIO/odom_imu`仍保持IMU输入频率；验证重定位成功前车辆里程计无输出，成功后开始输出，LOST后立即停止；验证SCAN局部地图中的点云没有被重复坐标变换。

当前Windows主机没有ROS Noetic构建环境，因此这里只能完成源码修改、测试代码编写和静态检查，不能声称ROS编译、rosbag或实车验证通过。

## 11. 非目标

- 不把正式IESKF改成逐IMU更新；
- 不在连续历史地图定位阶段增加滑窗BA；
- 不改变BTC重定位、空间块加载或只读地图算法；
- 不在Voxel-SLAM中硬编码机器狗外参；
- 不引入ROS2或`ros1_bridge`；
- 不修改原版SCAN-Planner核心算法，只提供配置和接口说明。
