# Voxel-SLAM-localization：建图、重定位与连续定位系统

ROS 2 Humble 迁移分支的构建、launch、rosbag2 和 RViz2 说明见 [`ROS2_HUMBLE.md`](ROS2_HUMBLE.md)。当前 `main` 分支仍是 ROS1 Noetic 版本。

## 1. 项目简介

本项目是在 Voxel-SLAM 基础上完成的建图与定位二次开发。工程保留原版的 LiDAR-IMU 前端、点到面迭代误差状态卡尔曼滤波（IESKF）、滑动窗口局部 BA、BTC 回环描述子和 GTSAM 优化能力，并将工作重点扩展到可复用地图上的重定位、连续定位及导航接入。

当前改造边界包括：

- 建图最终优化完成后，导出只读的 v2 空间体素块地图包；
- 提供 BTC 自动重定位与 RViz 人工初始化两条独立通道，统一求解临时里程计坐标系到历史地图坐标系的变换；
- 通过规则空间块索引、后台增量加载和不可变快照完成连续 `in-map tracking`，并在失效后回到局部里程计重新定位；
- 在权威激光后验之外提供独立的 IMU 高频均值预测，输出导航可用里程计，并通过适配器组合车体外参。

本文档描述当前仓库实际存在的配置、launch、可执行程序、ROS 话题和 TF 责任。建图与定位共用 `voxelslam` 可执行程序，由 `General/mode` 选择运行模式。

## 2. 主要功能

- LiDAR-IMU 建图、滑动窗口局部 BA、BTC 回环检测与最终全局优化；
- 最终优化后事务式导出 v2 地图，或使用 `build_spatial_map` 将已有 v1 地图转换为 v2；
- 带可选 x/y 先验范围、失败扩展和全局回退的 BTC 自动重定位；
- RViz **2D Pose Estimate** 人工初始化及连续多帧局部地图验证；
- 基于固定空间块的增量加载、前向预取、卸载滞回和地图缺口保护；
- 连续定位质量检查、`LOST` 恢复和只读历史地图保护；
- `/localization/state` 与 `/localization/valid` 状态接口；
- `/LIO/odom_imu` 高频 IMU 里程计、`/LIO/odom_vehicle` 车体里程计及 SCAN-Planner 接口适配；
- 轨迹误差、地图哈希、空间块耗时和内存统计工具。

## 3. 环境与编译

完整构建和 rosbag 集成验证需要：

- Ubuntu 20.04；
- [ROS Noetic](http://wiki.ros.org/ROS/Installation)；
- [PCL 1.10](https://pointclouds.org/)；
- [Eigen 3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page)；
- [GTSAM 4.0.3](https://github.com/borglab/gtsam/releases?page=2)；
- Boost `filesystem` 与 `system`；
- [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)。

先将当前仓库检出到 `~/catkin_ws/src/Voxel-SLAM`。建议先让 `rosdep` 补齐可解析的 ROS/系统依赖，再从 catkin 工作空间编译：

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

Windows 环境只能执行文档契约、Git 差异和不依赖 ROS/PCL/GTSAM 的 Python 测试，不能代替 Ubuntu 构建、C++ 测试和 rosbag 验证。

## 4. 完整运行流程

以下流程以 Livox Mid-360 为例。输入话题默认来自 `VoxelSLAM/config/mid360.yaml` 和 `VoxelSLAM/config/localization_mid360.yaml` 中的 `/livox/lidar`、`/livox/imu`；实际传感器或 rosbag 必须与配置一致。

**多终端环境说明：** 以下每个新终端都应先执行 `source ~/catkin_ws/devel/setup.bash`；若已写入 `~/.bashrc`，可省略该步骤。本说明覆盖后续建图、定位和导航接入中的全部终端。

### 4.1 建图配置

编辑 `VoxelSLAM/config/mid360.yaml`：

```yaml
General:
  mode: mapping
  map_export_path: "/absolute/path/to/v2_map"

MapBlock:
  size_x: 20.0
  size_y: 20.0
  size_z: 10.0
  fusion_leaf_size: 0.1
  root_voxel_size: 1.0
  max_layer: 2
  min_eigen_value: 0.0025
  save_display_cloud: false
```

`map_export_path` 必须指向尚不存在的目录，父目录必须已经存在。`MapBlock` 参数将写入 `manifest.yaml`，成为地图格式的一部分；定位时只能从清单读取，不能由定位 YAML 覆盖。

### 4.2 运行建图并导出 v2 地图

终端 1 启动建图节点，并保持该前台进程运行：

```bash
roslaunch voxel_slam vxlm_mid360.launch
```

终端 2 播放建图数据：

```bash
rosbag play mapping.bag --pause
```

`--pause` 会让 rosbag 启动后保持暂停。确认终端 1 中的节点已就绪，再回到终端 2 按空格键解除暂停。rosbag 播放结束后保持终端 1 的建图节点运行，并从另一终端 3 触发最终全局优化和地图导出：

```bash
rosparam set finish true
```

程序只在最终优化成功后写入完整 v2 地图包。若关键帧不足、没有生成有效平面空间块或写入校验失败，导出会失败并清理临时输出，不保留可被误用的空地图目录。

### 4.3 转换已有 v1 地图（可选）

定位程序只接受 v2 地图。Ubuntu 编译完成后，可执行：

```bash
rosrun voxel_slam build_spatial_map \
  --input /absolute/path/to/v1_map \
  --output /absolute/path/to/v2_map \
  --size-x 20 --size-y 20 --size-z 10 \
  --fusion-leaf 0.1 --root-voxel 1.0 \
  --max-layer 2 --min-eigen 0.0025
```

输入与输出不能相同、不能互相嵌套，输出目录不能预先存在。转换结束后，工具会重新读取索引并校验全部空间块；校验失败时不保留不完整输出。

### 4.4 启动定位

可以在 `VoxelSLAM/config/localization_mid360.yaml` 中设置 `Localization/map_path`，也可以由 launch 参数覆盖：

终端 1 启动定位节点，并保持该前台进程运行：

```bash
roslaunch voxel_slam vxlm_localization_mid360.launch \
  map_path:=/absolute/path/to/v2_map
```

终端 2 播放定位数据：

```bash
rosbag play localization.bag --pause
```

确认终端 1 中的定位节点已就绪后，在终端 2 按空格键解除暂停。

launch 会加载定位配置，强制 `General/mode=localization`，并保持 `General/map_export_path` 为空。地图清单、关键帧、BTC 数据库和空间块索引全部通过后，状态机进入 `LOCAL_ODOMETRY`。

### 4.5 完成初始重定位

默认配置启用 BTC 自动重定位。系统先在临时坐标系中积累当前 Session，形成查询关键帧，再执行 BTC 候选检索、平面 ICP 和当前 Session 对齐；成功后自动进入局部地图准备阶段。

若自动通道未成功，或需要人工指定位置，可在 RViz 中使用 **2D Pose Estimate**。配置项 `rviz_initialpose_enabled: true` 时，该入口始终可用。输入的 x/y/yaw 会与当前 IMU 的 roll/pitch、最近历史关键帧的 z 组合，并经过连续局部地图匹配验证；它不会修改 BTC 候选范围。

仓库自带的 `VoxelSLAM/rviz_cfg/back.rviz` 默认 RViz Fixed Frame 是 `camera_init`，因此默认流程无需额外发布 `map -> camera_init` TF。默认且推荐的 frame 是 `camera_init`。`map` 和 `/map` 仅作为数值坐标与 `camera_init` 完全相同的兼容别名；`InitialPoseRequestBuffer` 不查询 TF，也不执行 `map -> camera_init` 变换。若外部系统中的 `map` 与 `camera_init` 存在非恒等变换，必须先把位姿转换到 `camera_init` 后再发布，不能直接使用 `map` 别名。

### 4.6 检查定位状态

```bash
rostopic echo /localization/state
rostopic echo /localization/valid
```

成功判据为：

- `/localization/state` 进入 `IN_MAP_TRACKING`；
- `/localization/valid` 为 `true`；
- `/map_scan` 和 `/map_path` 持续更新；
- 启用高频输出时，`/LIO/odom_imu` 持续发布且时间戳随 IMU 前进。

定位过程中也可能看到 `WAITING_FOR_MAP`、`LOCAL_ODOMETRY`、`RELOCALIZING`、`ALIGNING`、`PREPARING_LOCAL_MAP` 或 `LOST`。只有 `IN_MAP_TRACKING` 对应有效地图定位。

### 4.7 接入 SCAN-Planner（可选）

本仓库没有 vendored SCAN-Planner；它是独立的外部工程，官方仓库为 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)。可以把它克隆到同一个 catkin 工作空间的 `src/` 后一起编译，此时每个新终端执行：

```bash
source ~/catkin_ws/devel/setup.bash
```

也可以在单独的 catkin 工作空间编译，并把它作为独立 overlay 叠加到 Voxel-SLAM 工作空间；此时每个新终端依次执行：

```bash
source ~/catkin_ws/devel/setup.bash
source ~/scan_planner_ws/devel/setup.bash
```

下面的接入步骤和参数来自接口静态核对，尚未在本仓库的 Windows 环境完成 Ubuntu/ROS 联调。用户自己的机器人描述需要提供静态 `aft_mapped -> base_link` 外参。每个前台进程应分别在独立终端启动并保持运行。

上游 `run.launch` 无条件设置 Go2 的 `robot_description`，并启动名为 `go2_robot_state_publisher` 的 `robot_state_publisher`。使用自己的 URDF 时，应先把该 launch 复制到用户 bringup 包并裁剪 Go2 model 段，再启动下面的用户机器人描述；不要重复启动 `robot_state_publisher`，也不要让上游 launch 覆盖用户的 `robot_description`。

#### 模式 A：沿用 4.4 已启动的定位会话

若 4.4–4.6 的 Voxel-SLAM 定位进程和传感器/rosbag 输入仍在运行，不得再次启动 `vxlm_localization_mid360.launch`，否则同名节点可能顶掉现有会话。保持现有 Voxel-SLAM 和传感器/rosbag 输入，仅新增或确认以下三个进程：

1. **用户 URDF**：确认用户工程的机器人描述及其 `robot_state_publisher` 已运行；若尚未运行，再执行：

```bash
roslaunch your_robot_description robot_description.launch
```

2. **adapter**：启动 `scan_planner_odom_adapter`：

```bash
roslaunch voxel_slam scan_planner_adapter.launch
```

3. **裁剪后的 SCAN-Planner**：使用上述已复制并裁剪的 launch 启动；命令中的 launch 名称按用户 bringup 包实际名称替换：

```bash
roslaunch your_robot_bringup scan_planner_voxelslam.launch \
  is_real_world:=true sensor_type:=lidar
```

#### 模式 B：从零启动实机完整栈

从零启动时，在五个独立终端中各启动一次以下进程，不要重复启动同名节点：

1. **传感器驱动/输入**：下面的 `your_lidar_driver` 和 launch 文件都是占位符，须按实际设备替换：

```bash
roslaunch your_lidar_driver your_lidar.launch
```

2. **用户 URDF**：

```bash
roslaunch your_robot_description robot_description.launch
```

3. **Voxel-SLAM 定位**：

```bash
roslaunch voxel_slam vxlm_localization_mid360.launch \
  map_path:=/absolute/path/to/v2_map
```

4. **adapter**：启动 `scan_planner_odom_adapter`：

```bash
roslaunch voxel_slam scan_planner_adapter.launch
```

5. **裁剪后的 SCAN-Planner**：

```bash
roslaunch your_robot_bringup scan_planner_voxelslam.launch \
  is_real_world:=true sensor_type:=lidar
```

`cloud_is_world` 与 `need_extrinsic` 是两个彼此独立的开关：前者决定输入点云是否还需由传感器系变换到世界系，后者决定输入位姿是否还需转换到 LiDAR 射线原点。本工程的 `/map_scan` 已在 `camera_init` 世界系，因此保持 `cloud_is_world=true`；但 `/LIO/odom_imu` 是 IMU 位姿，作为 `sensor_pose_topic` 时仍须保持 `need_extrinsic=true`，用 LiDAR→IMU 外参得到射线原点。

默认 Mid-360 外参平移为 `[-0.011, -0.02329, 0.04412]`、旋转为单位旋转。SCAN-Planner 当前在 `src/planner/plan_env/src/grid_map.cpp` 的 `mp_.lidar_extrinsic_` 中定义该矩阵；它必须与 `VoxelSLAM/config/localization_mid360.yaml` 的 `General/extrinsic_tran` 和 `General/extrinsic_rota` 一致。如果更换安装方式，两个工程必须同步修改。复制后的 `run.launch`/`advanced_param.xml` 需要使用等价设置：

```text
body_pose_topic=/LIO/odom_vehicle
sensor_pose_topic=/LIO/odom_imu
cloud_topic=/map_scan
grid_map/frame_id=camera_init
grid_map/cloud_is_world=true
grid_map/need_extrinsic=true
```

ROS1 launch 关键项为：

```xml
<arg name="cloud_topic" value="/map_scan"/>
<arg name="cloud_is_world" value="true"/>
<param name="grid_map/frame_id" value="camera_init" type="str"/>
<param name="grid_map/need_extrinsic" value="true" type="bool"/>
```

## 5. 建图与地图准备原理

建图模式沿用 Voxel-SLAM 的 LiDAR-IMU 前端、点到面 IESKF、滑动窗口局部 BA、BTC 回环和全局优化。`finish` 触发最终优化后，程序使用优化后的关键帧位姿把历史点云融合到统一世界坐标系，再按 `MapBlock` 的固定尺寸切分为静态空间块。空间块内保存根体素、静态八叉树节点和平面信息；发布完成后作为不可变定位地图使用。

v2 地图包的组成和职责如下：

- `manifest.yaml`：格式标识与版本、坐标系、关键帧/BTC 数量、BTC 配置指纹以及不可覆盖的空间块参数；
- `keyframes.csv`：关键帧编号、时间戳和世界系位姿，用于索引、先验范围筛选和最近高度查询；
- `keyframes/`：关键帧点云，保留兼容数据并支持地图转换；
- `btc_frames.bin`：历史 BTC 帧与描述子数据库；
- `btc_planes/`：与 BTC 历史锚点对应的平面点云，供几何验证和 ICP 精修；
- `spatial_blocks.idx`：非空规则块的哈希索引、包围盒、数据偏移、长度及校验信息；
- `spatial_blocks.bin`：融合后的不可变空间块二进制数据。

定位加载地图时会验证清单、BTC 数据与空间块索引的一致性。运行期不会修改磁盘地图包，也不会逐帧遍历全部关键帧或重新构建整张体素地图。

## 6. 重定位技术方案

### 6.1 双通道重定位总览

历史地图加载成功后，系统先进入 `LOCAL_ODOMETRY`：LiDAR-IMU 前端在临时里程计坐标系 `O` 中运行点到面 IESKF，并保留原有滑窗 BA，积累一段连续的当前 Session 轨迹。此时系统知道相对运动，但还不知道临时坐标系 `O` 在历史地图坐标系 `W` 中的位置。重定位的目标是求出并应用 `T_W_O`。

系统提供两条相互独立、最终汇合的初始化通道：

```text
            加载历史地图
                 ↓
  LOCAL_ODOMETRY（临时激光惯性里程计）
                 ↓
 ┌───────────────┴────────────────┐
 │                                │
BTC 自动重定位通道           RViz 人工初始化通道
 │                                │
BTC + 平面 ICP + 会话对齐     人工位姿 + 局部地图连续验证
 │                                │
 └───────────────┬────────────────┘
                 ↓
           应用 T_W_O
                 ↓
       PREPARING_LOCAL_MAP
                 ↓
        加载邻域历史空间块
                 ↓
         IN_MAP_TRACKING
```

两条通道的输入和验证方式不同：BTC 从当前 Session 点云生成描述子并通过 ICP/GTSAM 验证，RViz 从人工先验补全六自由度位姿并通过连续静态地图匹配验证。它们不会互相修改输入，但最终都产生 `T_W_O` 并进入同一空间块定位前端。

### 6.2 BTC 自动重定位

BTC 通道不需要操作人员给出当前位置。系统将局部里程计窗口内的多帧点云统一变换到当前锚点坐标系，生成平面点云和 BTC 三角形描述子，再与地图包中的历史 BTC 数据库匹配。

当 `use_prior_pose_for_btc: true` 时，历史关键帧位姿索引先查询 `btc_prior_pose.x/y` 周围 `btc_search_radius` 内的关键帧，只允许对应 BTC 帧参加描述子投票。候选筛选不限制 yaw；失败后可逐级扩大到 `btc_max_search_radius`，再按 `btc_fallback_to_global` 回退到全局搜索。当 `use_prior_pose_for_btc: false` 时，先验位置和所有半径/回退项均不参与筛选，BTC 直接使用全部历史帧。

候选命中后不会立即宣布定位成功。程序先检查 BTC 分数与变换有效性，再调用 `icp_normal` 对当前平面点云和候选历史平面点云进行几何验证与精修，并检查可观性；随后通过 GTSAM Levenberg-Marquardt 优化把当前 Session 一致地对齐到历史地图。

### 6.3 BTC 自动重定位完整链路

定位与建图使用相同的 BTC 描述子生成方法。默认 `LocalBA/win_size: 10`，系统收集 10 个经过 IMU 去畸变、IESKF 更新和局部 BA 优化的 `ScanPose`，将点云统一变换到窗口最后一帧坐标系，形成临时 BTC 查询关键帧并调用 `GenerateSTDescs()`。查询关键帧只用于重定位，不会写入历史 BTC 数据库或静态空间块地图。

```text
局部里程计输出 ScanPose（默认积累 10 帧）
        ↓
点云统一变换到最后一帧坐标系并融合
        ↓
调用 GenerateSTDescs() 生成当前 BTC 查询描述子和当前平面点云
        ↓
在允许的历史 BTC 帧集合中进行描述子检索
        ↓
获得候选历史锚点 H 以及初始相对位姿 T_H_C
        ↓
当前平面点云与历史平面点云执行 icp_normal
        ↓
得到经过几何验证和精修的 T_H_C
        ↓
固定历史锚点，使用 GTSAM 优化当前局部里程计 Session
        ↓
计算 T_W_O，并将当前状态和当前 Session 变换到历史地图坐标系
        ↓
进入 PREPARING_LOCAL_MAP，加载当前位置附近的历史空间块
        ↓
核心空间块就绪
        ↓
进入 IN_MAP_TRACKING
```

坐标系和变换定义如下：

- `W`：历史地图坐标系；
- `O`：当前临时局部里程计坐标系；
- `H`：BTC 匹配到的历史锚点帧；
- `C`：当前 BTC 查询窗口的最后一帧，即当前查询锚点；
- `T_H_C`：当前查询锚点 `C` 在历史锚点 `H` 坐标系下的相对位姿；
- `T_W_O`：当前临时里程计坐标系 `O` 在历史地图坐标系 `W` 下的位姿。

利用历史锚点的已知地图位姿 `T_W_H` 和当前锚点的局部里程计位姿 `T_O_C`：

```text
T_W_C = T_W_H × T_H_C
T_W_O = T_W_C × inverse(T_O_C)
```

这里的 GTSAM 优化不会重新优化整张历史地图。历史锚点作为固定先验保持不动，优化变量主要是当前 Session 的位姿；约束由当前 Session 内部的连续里程计约束和 BTC/ICP 提供的跨 Session 约束组成。优化目标是让整段当前局部轨迹一致地对齐到历史地图并求出 `T_W_O`。全局校正会一次性作用于 `x_curr`、滑窗状态、缓存轨迹和高频预测后验快照。

### 6.4 RViz 人工初始化

当 `rviz_initialpose_enabled: true` 时，RViz **2D Pose Estimate** 是始终可用的独立入口，它跳过 BTC，也不修改 BTC 候选范围。系统按以下方式补全六自由度初始位姿：

默认且推荐的 frame 是 `camera_init`。`map` 和 `/map` 仅作为数值坐标与 `camera_init` 完全相同的兼容别名；`InitialPoseRequestBuffer` 不查询 TF，也不执行 `map -> camera_init` 变换。若外部系统中的 `map` 与 `camera_init` 存在非恒等变换，必须先把位姿转换到 `camera_init` 后再发布，不能直接使用 `map` 别名。

- x、y、yaw 来自 `/initialpose`；
- roll、pitch 来自当前 IMU 姿态；
- z 来自 XY 平面中距离输入位置最近的历史关键帧；
- 当前局部里程计中的传感器位姿用于反求 `T_W_O`。

人工输入不会直接令定位有效。系统应用变换后先加载输入位置附近的核心空间块，再让实时扫描与静态历史平面地图执行点到面 IESKF。只有连续 `direct_pose_validation_scans` 帧同时满足匹配点数、平均点到面残差和可观性阈值，才进入 `IN_MAP_TRACKING`；任一坏帧会清零连续成功计数。超过 `direct_pose_validation_timeout_sec` 仍未通过时，本次请求被拒绝，临时状态被清理，状态机返回 `LOCAL_ODOMETRY`。新的 RViz 请求会替换尚未完成的旧请求。

### 6.5 两条通道的公共汇合路径

两条通道只在 `T_W_O` 的来源和验证方式上不同。通过各自安全检查后，它们都进入 `PREPARING_LOCAL_MAP`：管理器以地图坐标系中的当前位置为中心枚举规则块坐标，通过 `spatial_blocks.idx` 的哈希索引查找非空块，并在后台从 `spatial_blocks.bin` 加载核心块。

第一批核心块尚未就绪时，前端只保留 IMU 传播，不把当前扫描插入历史地图，也不使用不完整局部地图产生激光后验。BTC 已完成描述子、ICP 和 Session 对齐验证，因此核心块就绪即可置为有效；RViz 仅提供人工先验，因此还必须通过连续局部地图验证。两条路径最终都以不可变空间块快照进入同一套连续定位前端。

## 7. 连续定位技术方案

进入 `IN_MAP_TRACKING` 后，系统不再向临时 Session 地图插入点，也不运行滑窗 BA。每帧按以下顺序处理：

1. 收集该帧时间范围内的 IMU，传播状态与协方差并完成点云运动补偿；
2. 把当前位置和基于速度、`prefetch_time_sec` 得到的预测位置转换为规则块坐标，枚举 `load_radius` 邻域；
3. 通过 `spatial_blocks.idx` 哈希索引查询非空块，后台线程只读取缺失块；
4. 为一次完整 IESKF 更新取得不可变空间块快照，从静态体素/八叉树平面构造点到面残差；
5. 迭代修正姿态、位置、速度和 IMU 偏置，并以匹配点数、平均绝对点到面残差、法向矩阵最小特征值和求解有限性评估质量。

车辆移动时只增量加载新进入范围的空间块；块超过更大的 `unload_radius` 后才卸载，形成加载/卸载滞回，避免边界反复抖动。系统不会逐帧扫描全部历史关键帧，也不会把全部历史点重新体素化。

若核心块仍在后台读取或出现地图缺口，本帧激光更新会回滚到本帧 IMU 传播状态，不使用残缺快照强行校正；缺口持续超过 `map_gap_timeout_sec`、连续坏帧达到 `lost_bad_scan_limit`、状态出现 NaN/Inf 或空间块损坏时，状态机进入 `LOST`。系统随后把 `/localization/valid` 置为 `false`，停止默认导航里程计输出，清理当前 Session 和运行期空间块状态，再返回 `LOCAL_ODOMETRY` 重新进行 BTC 自动重定位或等待 RViz 输入。磁盘地图包始终只读。

## 8. IMU 高频里程计与导航接口

同一条 IMU 消息进入两条用途不同的路径，并不表示同一个滤波器被重复更新。

权威路径维护系统正式状态与协方差。IMU callback 先把消息写入正式缓冲；主线程在一帧激光及其时间范围内的 IMU 齐备后，批量执行状态/协方差传播、点云去畸变和点到面 IESKF 迭代更新。建图或 `LOCAL_ODOMETRY` 中的滑窗 BA 可以写回最新状态；`IN_MAP_TRACKING` 只使用不可变历史空间块完成 IESKF，不运行 BA。这条路径产生唯一的权威后验。

独立的 `ImuRatePredictor` 只维护短时状态均值副本。它从最近权威后验 `(R, p, v, b_g, b_a, g)` 出发，对新 IMU 做前向传播：

```text
ω = gyro - b_g
a_world = R · (scale · accel - b_a) + g
R ← R · Exp(ω·dt)
p ← p + v·dt + 0.5·a_world·dt²
v ← v + a_world·dt
```

该预测器不传播新的权威协方差，不参与激光去畸变或点到面残差计算，也不写回正式 IESKF。它发布 `/LIO/odom_imu` 以及相同传感器时间戳的动态 TF `camera_init -> aft_mapped`。

每次可靠扫描后验、局部 BA 写回或全局重定位校正后，系统都会刷新预测器。刷新会保留并重放时间戳晚于后验的 IMU 尾队列，使输出从新后验连续追赶到最新 IMU；尾队列不足以完整重放时，高频链不接管 TF，由扫描频率 TF 保底。

`max_prediction_sec` 限制最近激光后验之后允许的纯 IMU 外推时间，默认 0.5 秒。超时后快速输出停止并等待新后验，避免把长时间惯性漂移交给规划器。定位模式默认还受 `publish_before_localized: false` 门控，只有 `/localization/valid=true` 才发布导航里程计。系统重置、进入 `LOST`、IMU 时间回退或节点退出都会清空预测状态。

独立的 `scan_planner_odom_adapter` 订阅 `/LIO/odom_imu`，从 TF 读取静态 `aft_mapped -> base_link`，组合车体位姿，并利用角速度与杆臂修正 `base_link` 原点的世界系线速度，发布 `/LIO/odom_vehicle`。适配器不发布动态 TF。

## 9. 参数、话题与 TF

关键文件：

- 建图配置：`VoxelSLAM/config/mid360.yaml`；
- 定位配置：`VoxelSLAM/config/localization_mid360.yaml`；
- 车体适配配置：`VoxelSLAM/config/scan_planner_adapter.yaml`；
- 建图 launch：`VoxelSLAM/launch/vxlm_mid360.launch`；
- 定位 launch：`VoxelSLAM/launch/vxlm_localization_mid360.launch`；
- 适配器 launch：`VoxelSLAM/launch/scan_planner_adapter.launch`。

建图与地图格式参数：

- `General/mode`：`mapping` 或 `localization`；
- `General/map_export_path`：最终优化后导出的 v2 目录，仅建图模式使用；
- `Localization/map_path`：定位读取的 v2 地图目录，可由定位 launch 的 `map_path:=...` 覆盖；
- `MapBlock/size_x|size_y|size_z`、`fusion_leaf_size`、`root_voxel_size`、`max_layer`、`min_eigen_value`：写入清单的固定地图参数。

重定位参数：

- `use_prior_pose_for_btc`、`btc_prior_pose.x/y`、`btc_search_radius`：启用并定义 BTC x/y 先验范围；
- `btc_expand_on_failure`、`btc_max_search_radius`、`btc_fallback_to_global`：失败扩展和全局回退；
- `rviz_initialpose_enabled`：是否接受 RViz `/initialpose`；
- `direct_pose_validation_scans`、`direct_pose_validation_timeout_sec`：人工位姿连续验证帧数和超时；
- `alignment_max_translation_m`、`alignment_max_rotation_deg`：拒绝不合理的 Session 到地图校正。

空间块与跟踪参数：

- `preload_radius`：重定位后必须先准备的核心半径；
- `load_radius`：当前位置和预测位置周围的目标加载半径；
- `unload_radius`：卸载滞回半径，必须大于 `load_radius`；
- `prefetch_time_sec`：沿当前速度预取的时间跨度；
- `map_gap_timeout_sec`、`preload_timeout_sec`：地图缺口与首次准备超时；
- `max_background_loads`：后台并行加载线程数；
- `tracking_min_matched_points`、`tracking_max_mean_abs_point_plane`、`tracking_min_normal_eigenvalue`、`lost_bad_scan_limit`：跟踪质量与失效阈值。

高频里程计参数：

- `imu_rate_output`：启用独立 IMU 高频预测；
- `imu_odom_topic`：默认 `/LIO/odom_imu`；
- `odom_frame`、`imu_child_frame`：默认 `camera_init`、`aft_mapped`；
- `publish_before_localized`：定位有效前是否允许发布，默认 `false`；
- `max_prediction_sec`：纯 IMU 外推上限，默认 0.5 秒。

主要 ROS 接口：

- `/localization/state`（`std_msgs/String`，锁存）：定位状态机当前状态；
- `/localization/valid`（`std_msgs/Bool`，锁存）：导航输出有效性门控；
- `/map_scan`（`sensor_msgs/PointCloud2`）：已变换到 `camera_init` 的当前配准点云，按激光帧率发布；
- `/map_path`（`sensor_msgs/PointCloud2`）：当前轨迹点云，frame 为 `camera_init`；
- `/LIO/odom_imu`（`nav_msgs/Odometry`）：IMU 频率地图系预测状态，父/子 frame 默认为 `camera_init`/`aft_mapped`；
- `/LIO/odom_vehicle`（`nav_msgs/Odometry`）：适配器组合静态安装外参后的 `base_link` 车体状态。

TF 责任保持为：

```text
camera_init -> aft_mapped -> base_link
```

Voxel-SLAM 发布动态 `camera_init -> aft_mapped`；用户的 URDF、`robot_state_publisher` 或静态 TF 发布器必须提供固定 `aft_mapped -> base_link`。不要同时发布相反方向的静态 TF，否则会让 `aft_mapped` 获得第二个父节点。适配器只读取该静态外参，不发布额外动态 TF。

## 10. 验证、评估与常见问题

Ubuntu 20.04 完整验证：

```bash
cd ~/catkin_ws
catkin_make
catkin_make run_tests
catkin_test_results
python3 -m unittest discover -s src/Voxel-SLAM/VoxelSLAM/test -p "*_test.py"
```

定位轨迹与运行期地图一致性评估：

```bash
cd ~/catkin_ws/src/Voxel-SLAM
python3 VoxelSLAM/scripts/evaluate_localization.py \
  --estimate estimate.tum \
  --reference reference.tum \
  --map-dir /absolute/path/to/v2_map \
  --runtime-log spatial_runtime.log
```

工具输出不进行事后对齐的地图坐标系 ATE RMSE、平移/旋转 RPE、逐文件及整目录 SHA-256、活动块和待加载块峰值、核心块就绪比例、加载/匹配/整帧耗时统计以及已加载静态块的估算内存峰值。定位前后两次地图哈希应完全一致。可在启动定位时附加日志重定向，例如 `roslaunch voxel_slam vxlm_localization_mid360.launch map_path:=/absolute/path/to/v2_map 2>&1 | tee spatial_runtime.log`，以保存 `SPATIAL_MAP active=...` 诊断行。

导航接口检查：

```bash
rostopic hz /LIO/odom_imu
rostopic hz /LIO/odom_vehicle
rostopic delay /LIO/odom_imu
rostopic echo -n 1 /localization/valid
rosrun tf tf_echo camera_init aft_mapped
rosrun tf tf_echo aft_mapped base_link
```

Windows 可执行的仓库检查：

```powershell
git diff --check
py -3.12 -m unittest discover -s VoxelSLAM/test -p "*_test.py"
```

常见问题：

- 定位启动即报地图错误：确认 `Localization/map_path` 指向完整 v2 包，且配置指纹、索引和二进制块未被修改；v1 地图先运行转换器。
- 长时间停留在 `LOCAL_ODOMETRY`：检查 BTC 是否启用正确先验范围；必要时等待半径扩展/全局回退，或使用 RViz 人工初始化。
- 停留在 `PREPARING_LOCAL_MAP`：检查地图路径、空间块校验、`preload_radius`、后台加载日志和 `preload_timeout_sec`。
- `/localization/valid` 为 `false`：默认不会输出导航里程计；先处理重定位或跟踪质量问题，不要绕过有效性门控。
- 只有 `/LIO/odom_imu`、没有 `/LIO/odom_vehicle`：检查适配器是否启动、`aft_mapped -> base_link` 静态 TF 的方向和 frame 名称，以及适配器节流告警。
- 高频输出约 0.5 秒后停止：检查新的激光权威后验是否持续到达，不应首先放大 `max_prediction_sec`。
- SCAN-Planner 中点云或射线原点异常：确认 `/map_scan` 是世界系点云并设置 `cloud_is_world=true`；若 `sensor_pose_topic=/LIO/odom_imu`，同时保持 `need_extrinsic=true`，并核对 Mid-360 外参与 Voxel-SLAM 配置一致。

## 11. 上游项目与致谢

本项目基于 HKU-MARS 的 [Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM) 开展二次开发，感谢原作者公开代码与研究成果：

- 论文：[Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System](https://arxiv.org/abs/2410.08935)；
- 演示：[YouTube 视频](https://youtu.be/Cg9W01aIUzE)；
- 竞赛：原版系统作为子系统参加 [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html)，获得激光雷达单会话赛道第 2 名；
- 竞赛：参加 [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI)，获得激光雷达-惯性赛道第 1 名。

上游仓库还提供 Livox Avia、HILTI 多会话、MARS 和 Mid-360 数据集示例；本 README 聚焦当前工程的建图、地图准备、重定位、连续定位与导航接入流程。
