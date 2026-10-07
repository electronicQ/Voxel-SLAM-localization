# Voxel-SLAM：完整、精确且通用的激光雷达-惯性 SLAM 系统

## 1. 项目简介

**Voxel-SLAM** 是一套完整、精确且通用的激光雷达-惯性 SLAM 系统，充分利用短期、中期、长期以及多地图数据关联。系统包含初始化、里程计、局部建图、回环检测和全局建图五个模块。

初始化模块能够在静止或运动的初始状态下提供精确的状态估计和局部地图；里程计模块估计当前状态并检测系统可能出现的发散；局部建图模块通过激光雷达-惯性束调整（BA）优化滑动窗口内的状态与局部地图；回环检测支持跨多个会话进行匹配；全局建图模块通过高效的分层全局束调整优化全局地图。系统总体结构如下：

<div align="center">
    <a href="https://youtu.be/Cg9W01aIUzE" target="_blank">
    <img src="./figure/systemoverview.png" width = 60% >
</div>

### 1.1 相关视频

**Voxel-SLAM** 的演示视频可在 [YouTube](https://youtu.be/Cg9W01aIUzE) 上观看。

### 1.2 相关工作

相关论文可在 [arXiv](https://arxiv.org/abs/2410.08935) 上查看。

### 1.3 竞赛成绩

Voxel-SLAM 曾作为子系统参加 [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html)，获得激光雷达单会话赛道第 **2** 名；参加 [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI)，获得激光雷达-惯性赛道第 **1** 名。

## 2. 环境依赖

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/ROS/Installation)
- [PCL 1.10](https://pointclouds.org/)
- [Eigen 3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [GTSAM 4.0.3](https://github.com/borglab/gtsam/releases?page=2)
- [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)

## 3. 编译

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/Voxel-SLAM
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. 运行 Voxel-SLAM

### 4.1 Livox Avia

在线重定位实验所使用的 rosbag 可从 [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErEznhkJzTxJiLuJ8AQDGS0BvCy6KsuaWF2D6cnx061GEQ?e=dMRSlf) 或 [Google Drive](https://drive.google.com/file/d/1LG46i0vreQrMZRap5tJkjKK4IX0t0zfC/view?usp=drive_link) 下载。

```bash
roslaunch voxel_slam vxlm_avia.launch
# 使用 --pause 可以保证不同实验从相同的 rosbag 时刻开始。
# 按空格键开始播放。
rosbag play compus_elevator.bag --pause
```

设备位于电梯内时，系统会持续尝试重新初始化，直到设备离开电梯。蓝色点云表示初始化阶段建立的地图。

rosbag 播放结束后，地图可能会像演示视频中一样存在不一致。执行以下命令启动最终的全局建图（全局束调整），以优化全局地图：

```bash
rosparam set finish true
```

### 4.2 HILTI 2023（多会话）

多会话实验 rosbag 可从 [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/Epp5AQ2Oq1VNhC6MIuCAtN4BJC9jx9VvuVx7VT_cdlvD0A?e=8mXYdc) 下载并进行快速测试。完整的 HILTI 2022 和 HILTI 2023 数据可从[官方网站](https://hilti-challenge.com/index.html)获取。

建议按照 `site1_handheld_5` 到 `site1_handheld_1` 的顺序播放 rosbag，否则 `site1_handheld_2` 和 `site1_handheld_3` 可能无法检测到回环。

启动前，请在 `hesai.yaml` 中配置以下参数。`#` 表示注释：

```yaml
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "# site1_handheld_5: 0.50,
               # site1_handheld_4: 0.45,
               # site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_${1-5}" # 当前播放的 rosbag 名称
is_save_map: 1 # 启用地图保存
```

首先运行 `site1_handheld_5`：

```bash
roslaunch voxel_slam vxlm_hesai.launch
rosbag play site1_handheld_5.bag --pause
```

加载 `site1_handheld_5`，然后运行 `site1_handheld_4`：

```bash
roslaunch voxel_slam vxlm_hesai.launch
rosbag play site1_handheld_4.bag --pause
```

加载 `site1_handheld_5` 和 `site1_handheld_4`，然后运行 `site1_handheld_3`：

```bash
roslaunch voxel_slam vxlm_hesai.launch
rosbag play site1_handheld_3.bag --pause
```

运行 `site1_handheld_2` 前不要忘记加载已有离线地图，此时 `hesai.yaml` 应类似如下配置：

```yaml
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "site1_handheld_5: 0.50,
               site1_handheld_4: 0.45,
               site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_2" # 当前播放的 rosbag 名称
is_save_map: 1 # 启用地图保存
```

加载 `site1_handheld_5`、`site1_handheld_4` 和 `site1_handheld_3`，然后运行 `site1_handheld_2`：

```bash
roslaunch voxel_slam vxlm_hesai.launch
rosbag play site1_handheld_2.bag --pause
```

加载前四个会话后运行 `site1_handheld_1`：

```bash
roslaunch voxel_slam vxlm_hesai.launch
rosbag play site1_handheld_1.bag --pause
```

地图可能仍会出现演示视频中的不一致。执行以下命令进行最终全局束调整：

```bash
rosparam set finish true
```

### 4.3 MARS 数据集

快速测试数据可从 [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/EpjsGW6coYlMvBWo8TlgJXoBttAuoocLi24V6kw-r_3A8w?e=vVB6RY) 下载，完整 MARS 数据集可从[官方网站](https://mars.hku.hk/dataset.html)获取。

```bash
roslaunch voxel_slam vxlm_avia_fly.launch
rosbag play HKisland03.bag --pause
```

该数据起始阶段点云为空，系统会持续尝试初始化，直到无人机上升到一定高度。

```bash
roslaunch voxel_slam vxlm_avia_fly.launch
rosbag play AMvalley03.bag --pause
rosparam set finish true
```

该序列较难检测到回环，请运行 GBA 以保证全局地图的一致性。

### 4.4 Livox Mid-360

该 rosbag 开始时运动速度较快，可从 [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErtuXCFhFrBErZxzS5vLASkBJEfgDB9R2CSCgKe8BwhneQ?e=zbr7NL) 下载。

```bash
roslaunch voxel_slam vxlm_mid360.launch
rosbag play jungle_challenge.bag --pause
```

### 4.5 定位

本节统一介绍定位地图的构建与转换、定位启动、双通道重定位、连续跟踪、结果评估以及导航接口。建图和定位使用同一个 `voxelslam` 可执行程序。

#### 4.5.1 导出定位地图

在建图配置中保持 `General/mode: mapping`，并在 `VoxelSLAM/config/mid360.yaml` 中设置 `General/map_export_path`。目标目录必须尚不存在，但其父目录必须已经存在。`MapBlock` 配置决定定位地图的固定空间划分：默认块尺寸为 20 m × 20 m × 10 m，块内融合分辨率为 0.1 m，根体素边长为 1 m，静态八叉树最大层数为 2。

正常运行建图，然后触发最终全局优化和地图导出：

```bash
roslaunch voxel_slam vxlm_mid360.launch
rosbag play mapping.bag --pause
rosparam set finish true
```

程序仅在最终优化完成后以事务方式写入 v2 地图包，避免留下不完整目录。主要内容如下：

- `manifest.yaml`：格式版本、坐标系、BTC 指纹、关键帧数量和不可修改的空间块参数；
- `keyframes.csv` 与 `keyframes/`：BTC 范围筛选、最近高度查询和兼容数据；
- `btc_frames.bin` 与 `btc_planes/`：BTC 重定位数据库；
- `spatial_blocks.idx`：非空空间块的哈希索引、包围盒、偏移、长度和校验信息；
- `spatial_blocks.bin`：融合后的不可变空间块、根体素、静态八叉树节点和平面数据。

如果最终关键帧过于稀疏，或 `MapBlock` 阈值导致无法生成任何有效平面空间块，导出会明确失败并清理临时目录，不会生成一个无法定位的空 v2 地图包。

定位不会遍历全部历史关键帧，也不会重建整张局部体素地图。它只枚举当前位置和预测位置附近的规则块坐标，通过索引进行平均常数时间查找，在后台加载缺失块，并以不可变快照供一次完整 IESKF 更新使用。

#### 4.5.2 将旧 v1 地图转换为 v2

定位程序只接受 v2 地图。已有 v1 地图可在 Ubuntu 编译完成后转换：

```bash
rosrun voxel_slam build_spatial_map \
  --input /absolute/path/to/v1_map \
  --output /absolute/path/to/v2_map \
  --size-x 20 --size-y 20 --size-z 10 \
  --fusion-leaf 0.1 --root-voxel 1.0 \
  --max-layer 2 --min-eigen 0.0025
```

输入目录和输出目录不能相同、不能互相嵌套，输出目录也不能预先存在。转换程序完成写入后会重新读取索引并校验全部空间块；校验失败时不会保留不完整的 v2 输出。

#### 4.5.3 启动地图定位

可以在 `VoxelSLAM/config/localization_mid360.yaml` 中设置 `Localization/map_path`，也可以通过 launch 参数传入地图路径：

```bash
roslaunch voxel_slam vxlm_localization_mid360.launch \
  map_path:=/absolute/path/to/map_package
rosbag play localization.bag --pause
```

`Localization/spatial_map` 只控制运行时策略：

- `preload_radius`：重定位后必须先加载完成的核心半径；
- `load_radius`：当前位置及预测位置周围的目标加载半径；
- `unload_radius`：卸载滞回半径，必须大于 `load_radius`；
- `prefetch_time_sec`：使用当前速度预测前方加载中心的时间；
- `map_gap_timeout_sec`：核心块持续未就绪时允许仅 IMU 传播的最长时间；
- `preload_timeout_sec`：首次局部地图准备超时；
- `max_background_loads`：并行磁盘加载线程数。

块尺寸、融合分辨率、根体素尺寸、八叉树层数和平面阈值必须以 `manifest.yaml` 为准，定位 YAML 不能覆盖这些格式参数。

当 `use_prior_pose_for_btc: true` 时，BTC 仅从 `btc_prior_pose.x/y` 周围 `btc_search_radius` 范围内的历史关键帧中选择候选。匹配失败后，搜索半径会逐步扩大到 `btc_max_search_radius`，随后可根据配置回退到全局搜索。候选筛选有意不使用 yaw 约束。

当 `use_prior_pose_for_btc: false` 时，先验位置以及半径、扩展和回退设置均不参与候选筛选，BTC 会直接进行全局搜索。

#### 4.5.4 BTC/RViz 双通道重定位

##### 双通道重定位总览

历史地图加载成功后，系统先进入 `LOCAL_ODOMETRY`：LiDAR-IMU 前端在临时里程计坐标系 `O` 中运行点到面 IESKF，并保留原有滑窗 BA，积累一段连续的当前 Session 轨迹。此时系统知道相对运动，但还不知道临时坐标系 `O` 在历史地图坐标系 `W` 中的位置。重定位的目标就是求出并应用 `T_W_O`。

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

##### BTC 自动重定位通道

BTC 通道不需要操作人员给出当前位置。系统将局部里程计窗口内的多帧点云统一变换到当前锚点坐标系，生成平面点云和 BTC 三角形描述子，再与地图包中的历史 BTC 数据库匹配。

当 `use_prior_pose_for_btc: true` 时，历史关键帧位姿索引先查询 YAML 先验 x/y 周围 `btc_search_radius` 内的关键帧，只允许这些关键帧对应的 BTC 帧参加描述子投票。候选筛选不限制 yaw；失败后半径逐级扩大到 `btc_max_search_radius`，并可按配置回退到全局搜索。当 `use_prior_pose_for_btc: false` 时，BTC 直接使用全部历史 BTC 帧。

BTC 命中候选后不会立即宣布定位成功，而是依次执行：

1. 检查 BTC 分数和输出变换是否有效；
2. 使用当前平面点云与候选历史平面点云执行 `icp_normal`，精修相对位姿并检查可观性；
3. 以历史锚点为固定先验、当前 Session 的相邻里程计为连续约束、BTC/ICP 结果为跨 Session 约束，使用 GTSAM Levenberg–Marquardt 优化当前 Session；
4. 从优化后的当前锚点计算 `T_W_O`，一次性修正 `x_curr`、滑窗状态、缓存轨迹以及高频预测后验快照。

因此，当前自动通道是“BTC 候选检索 → 平面 ICP 验证与精修 → GTSAM LM 会话对齐”，不再使用原方案的 BnB 或 SE-Sync。跨 Session 约束通过并完成全局校正后，状态由 `RELOCALIZING`、`ALIGNING` 进入 `PREPARING_LOCAL_MAP`；核心空间块就绪后进入 `IN_MAP_TRACKING`，并令 `/localization/valid=true`。

##### 面试速记：BTC 重定位完整链路

定位阶段与建图阶段使用相同的 BTC 描述子生成方法。默认 `LocalBA/win_size: 10`，系统收集 10 个经过 IMU 去畸变、IESKF 更新和局部 BA 优化的 `ScanPose`，将它们的点云统一变换到窗口最后一帧坐标系，形成一个临时 BTC 查询关键帧并调用 `GenerateSTDescs()`。该查询关键帧只用于重定位，不会写入历史 BTC 数据库或静态空间块地图。

```text
局部里程计输出 ScanPose（默认积累 10 帧）
        ↓
点云统一变换到最后一帧坐标系并融合
        ↓
生成当前 BTC 查询描述子和当前平面点云
        ↓
在允许的历史 BTC 帧集合中进行描述子检索
        ↓
获得候选历史锚点 H 以及初始相对位姿 T_H_C
        ↓
当前平面点云与历史平面点云执行 ICP
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

坐标系和变换含义如下：

- `W`：历史地图坐标系；
- `O`：当前临时局部里程计坐标系；
- `H`：BTC 匹配到的历史锚点帧；
- `C`：当前 BTC 查询窗口的最后一帧，即当前查询锚点；
- `T_H_C`：当前查询锚点 `C` 在历史锚点 `H` 坐标系下的相对位姿；
- `T_W_O`：当前临时里程计坐标系 `O` 在历史地图坐标系 `W` 下的位姿。

利用历史锚点的已知地图位姿 `T_W_H` 和当前锚点的局部里程计位姿 `T_O_C`，可写成：

```text
T_W_C = T_W_H × T_H_C
T_W_O = T_W_C × inverse(T_O_C)
```

这里的 GTSAM 优化不是重新优化整张历史地图。历史锚点作为固定先验保持不动，优化变量主要是当前 Session 的位姿；图中的约束由当前 Session 内部的连续里程计约束和 BTC/ICP 提供的跨 Session 约束组成。优化的目的，是让整段当前局部轨迹一致地对齐到历史地图并求出 `T_W_O`。

##### RViz 人工初始化通道

当 `rviz_initialpose_enabled: true` 时，RViz 的 **2D Pose Estimate** 是一条始终可用的独立通道，它跳过 BTC，也不会用来修改 BTC 候选范围。系统按以下方式补全六自由度初始位姿：

- x、y 和 yaw 取自 `/initialpose`，表示车辆当前在地图坐标系中的位姿；
- roll 和 pitch 取自当前 IMU 姿态；
- z 取自 XY 平面内距离输入位置最近的历史关键帧高度；
- 当前局部里程计中的传感器位姿用于反求并应用 `T_W_O`。

人工点击可能存在误差，因此 RViz 位姿本身不会直接令定位有效。应用变换后系统先加载输入位置附近的核心空间块，再用实时扫描与静态历史平面地图执行点到面 IESKF。只有连续 `direct_pose_validation_scans` 帧同时满足匹配点数、平均点到面残差和可观性阈值，状态才由 `PREPARING_LOCAL_MAP` 转为 `IN_MAP_TRACKING`。任一坏帧会把连续成功计数清零；超过 `direct_pose_validation_timeout_sec` 仍未通过则拒绝本次位姿、清理临时状态并返回 `LOCAL_ODOMETRY`。新的 RViz 请求会替换尚未完成的旧请求。

##### 两条通道的公共汇合路径

两条通道的差别只在于 `T_W_O` 的来源和验证方式；变换一旦通过各自的安全检查，后续都使用同一个空间块地图管理器。`PREPARING_LOCAL_MAP` 期间，管理器以地图坐标系下的当前位置为中心枚举附近规则块坐标，通过 `spatial_blocks.idx` 哈希索引查找非空块，并在后台从 `spatial_blocks.bin` 加载核心块。第一批核心块尚未就绪时只保留 IMU 传播，不把当前扫描插入历史地图，也不使用不完整局部地图产生激光后验。

BTC 已经经过描述子、ICP 和会话对齐验证，所以核心块就绪后即可置为有效定位；RViz 只提供人工先验，因此核心块就绪后还必须完成连续多帧局部地图匹配验证。两条路径最终都以不可变空间块快照进入同一套连续定位前端。

#### 4.5.5 连续定位状态与自动恢复

##### 连续 in-map tracking

进入 `IN_MAP_TRACKING` 后，系统不再向临时 Session 地图插入点，也不运行滑窗 BA。每帧处理顺序如下：

1. 收集该帧激光时间范围内的 IMU，传播状态和协方差，并完成点云运动补偿；
2. 根据当前位姿和速度预测位置请求 `load_radius` 内的空间块，同时预取车辆前方块；
3. 从当前不可变空间块快照中的静态体素/八叉树平面查询点对应平面，构造点到面残差；
4. 通过迭代误差状态卡尔曼滤波修正姿态、位置、速度和 IMU 偏置；
5. 使用匹配点数、平均绝对点到面残差、法向矩阵最小特征值以及求解结果有限性判断本帧质量。

车辆移动时只增量加载新进入范围的空间块，并在块超出更大的 `unload_radius` 后卸载，通过加载/卸载滞回避免边界抖动。系统不会逐帧遍历全部历史关键帧，也不会把所有历史点重新体素化或重建整张局部地图。若下一位置所需核心块仍在后台读取，本帧激光更新会回滚到 IMU 传播状态并等待地图，不使用残缺地图强行更新。

##### 失效检测与自动恢复

程序发布以下锁存状态话题：

- `/localization/state`（`std_msgs/String`）：可能为 `WAITING_FOR_MAP`、`LOCAL_ODOMETRY`、`RELOCALIZING`、`ALIGNING`、`PREPARING_LOCAL_MAP`、`IN_MAP_TRACKING` 或 `LOST`；
- `/localization/valid`（`std_msgs/Bool`）：只有 BTC 对齐和核心地图准备成功，或者 RViz 位姿通过连续局部地图验证后才为 `true`。

连续坏帧达到阈值、状态出现 NaN/Inf、核心块损坏或地图缺口持续超过 `map_gap_timeout_sec` 时，状态机进入 `LOST`。系统将 `/localization/valid` 置为 `false`，停止默认导航里程计输出，清理当前 Session 和运行时空间块状态，然后返回 `LOCAL_ODOMETRY`，重新进行 BTC 自动重定位或等待新的 RViz 位姿；磁盘地图包和历史 BTC 数据始终保持只读。

#### 4.5.6 定位结果与实时性评估

使用以下命令，将 TUM 格式的估计轨迹与地图坐标系下的 TUM 真值轨迹进行比较，并检查定位过程中地图包是否发生变化：

```bash
python3 VoxelSLAM/scripts/evaluate_localization.py \
  --estimate estimate.tum \
  --reference reference.tum \
  --map-dir /absolute/path/to/map_package \
  --runtime-log spatial_runtime.log
```

评估工具输出以下结果：

- 不进行事后对齐的地图坐标系 ATE RMSE。
- 平移 RPE 和旋转 RPE。
- 地图包中每个文件的 SHA-256。
- 与路径相关且结果确定的整目录 SHA-256。
- 活动块数和待加载块数峰值、核心块就绪比例。
- 单块加载、静态地图匹配和整帧处理耗时的平均值、P95 与最大值。
- 已加载静态空间块数据的估算内存峰值。

定位时程序会输出形如 `SPATIAL_MAP active=...` 的诊断行，可用 `roslaunch ... 2>&1 | tee spatial_runtime.log` 保存。论文实验至少应记录加载耗时、匹配耗时、整帧耗时的平均值、P95、最大值，以及内存峰值。建议在定位前后分别运行一次评估工具，两次输出的所有地图哈希值必须完全一致。

#### 4.5.7 Ubuntu 验证与 Windows 可执行检查

完整构建和 C++ 测试必须在安装 ROS Noetic、PCL、GTSAM 和 Livox 驱动的 Ubuntu 20.04 环境中执行：

```bash
cd ~/catkin_ws
catkin_make
catkin_make run_tests
catkin_test_results
python3 -m unittest VoxelSLAM/test/evaluate_localization_test.py
```

随后分别使用建图 rosbag 导出 v2 地图、使用定位 rosbag 验证 BTC 和 RViz 两条初始化路径，并检查 `/localization/state`、`/localization/valid`、轨迹误差、实时性日志和地图哈希。

Windows 当前只能执行不依赖 ROS/PCL/GTSAM 的检查，不能代替 Ubuntu 编译和 rosbag 集成测试：

```powershell
python -m unittest VoxelSLAM/test/evaluate_localization_test.py
git diff --check
```

#### 4.5.8 高频里程计与 ROS1 SCAN-Planner 接入

同一条 IMU 消息会进入两条用途不同的处理路径。两条路径都使用该测量并不等于重复更新同一个滤波器：正式路径维护权威状态和协方差，高频路径只维护一个独立的短时状态均值副本，用于把最新可靠激光后验前向传播到最近 IMU 时间。

##### 正式 IESKF 路径

IMU callback 首先在互斥锁保护下将消息写入 `imu_buf`。主线程等待一帧激光及其时间范围内的 IMU 数据齐备后，批量执行 IMU 状态与协方差传播、点云去畸变和点到面 IESKF 迭代更新。这条路径生成系统唯一的权威后验；建图或重定位前的 `LOCAL_ODOMETRY` 还可能由滑窗 BA 写回最新状态，而连续 `IN_MAP_TRACKING` 只使用不可变历史空间块地图进行 IESKF 更新，不运行 BA。

##### IMU 高频预测路径

该 callback 本身运行在独立的单线程 IMU callback queue 上；完成正式缓冲后，它继续把时间戳、角速度和线加速度送入 `ImuRatePredictor`。预测器从最近权威后验 `(R, p, v, b_g, b_a, g)` 出发，对每条新 IMU 执行短时均值传播：

```text
ω = gyro - b_g
a_world = R · (scale · accel - b_a) + g
R ← R · Exp(ω·dt)
p ← p + v·dt + 0.5·a_world·dt²
v ← v + a_world·dt
```

快速预测不传播一套新的权威协方差，不参与激光去畸变或点到面残差计算，也绝不写回正式 IESKF。它发布 `/LIO/odom_imu` 和同时间戳的动态 TF `camera_init -> aft_mapped`，时间戳直接取 IMU 传感器时间而不是 `ros::Time::now()`，因此规划器能以接近 IMU 的频率读取当前预测位姿。

##### 后验快照刷新与输出门控

每次扫描 IESKF 得到可靠后验、滑窗 BA 写回最新状态或全局重定位校正成功后，系统都会用该后验刷新预测器。刷新时会保留并重放时间戳晚于后验的 IMU 尾队列，使输出从新的后验连续追赶到最新 IMU，而不是把输出时间倒退到激光时刻。如果尾队列不足以完整重放，快速链不会接管 TF，扫描频率 TF 会继续保底。

定位模式默认只在 `/localization/valid=true` 后发布高频导航里程计。`max_prediction_sec` 限制最近激光后验之后允许的纯 IMU 外推时长，默认 0.5 秒；超过时停止快速输出并等待新后验，避免把长时间惯性漂移交给规划器。系统重置、进入 `LOST`、IMU 时间回退或节点退出都会清空预测状态。高频功能被关闭、尚无可用快照或快照重放被拒绝时，由扫描频率 TF 继续发布 `camera_init -> aft_mapped`，避免两个发布器同时争用同一动态 TF。

##### 车辆里程计适配

`/LIO/odom_imu` 描述的是 IMU 在地图坐标系中的状态。独立的 `scan_planner_odom_adapter` 从 TF 读取静态 `aft_mapped -> base_link` 安装外参，组合机体位姿，并使用角速度与杆臂计算 `base_link` 原点的世界系线速度，发布 `/LIO/odom_vehicle`。适配器不发布额外动态 TF，因而 TF 树仍由 Voxel-SLAM 的动态 `camera_init -> aft_mapped` 和用户 URDF 提供的静态 `aft_mapped -> base_link` 组成。

`Odometry` 下的配置默认值为：

```yaml
Odometry:
  imu_rate_output: true
  imu_odom_topic: "/LIO/odom_imu"
  odom_frame: "camera_init"
  imu_child_frame: "aft_mapped"
  publish_before_localized: false
  max_prediction_sec: 0.5
```

主要接口如下：

- `/LIO/odom_imu`：IMU 频率的地图系预测状态。pose 为 `camera_init` 中的 IMU 位姿，线速度按本项目导航约定表达在世界系，角速度表达在 IMU/body 系。
- TF `camera_init -> aft_mapped`：与 `/LIO/odom_imu` 使用完全相同的传感器时间戳。高频链尚未就绪或被关闭时，保留扫描频率 TF 回退。
- `/LIO/odom_vehicle`：独立适配节点根据静态 `aft_mapped -> base_link` 外参组合得到的机体里程计，频率与 `/LIO/odom_imu` 相同。
- `/map_scan`：已经变换到 `camera_init` 的当前配准点云，不是 `laser_link` 下的原始点云，频率为激光雷达帧率。
- `/localization/state`、`/localization/valid`：定位状态和导航输出有效性门控。

本工程不会发布机器人安装外参。用户的 URDF、`robot_state_publisher` 或静态 TF 发布器必须提供从 `aft_mapped` 到 `base_link` 的固定变换，使 TF 树保持为 `camera_init -> aft_mapped -> base_link`；不要同时发布方向相反、让 `aft_mapped` 获得第二个父节点的静态 TF。

推荐启动顺序：

```bash
# 1. 启动机器人 URDF/robot_state_publisher，提供 aft_mapped -> base_link
roslaunch your_robot_description robot_description.launch

# 2. 启动 Voxel-SLAM 定位
roslaunch voxel_slam vxlm_localization_mid360.launch \
  map_path:=/absolute/path/to/map_package

# 3. 启动车辆里程计适配器
roslaunch voxel_slam scan_planner_adapter.launch

# 4. 按下述接口修改 SCAN-Planner 的 launch 后启动
roslaunch scan_planner run.launch is_real_world:=true sensor_type:=lidar
```

ROS1 SCAN-Planner 的 `main` 分支已经默认使用 `/LIO/odom_vehicle` 和 `/LIO/odom_imu`，但其实机 launch 仍把点云固定为 `/LIO/clouds_lidar`、`cloud_is_world=false`，并令 `need_extrinsic=is_real_world`。由于本工程输出的 `/map_scan` 已经在世界系，直接使用原 launch 会对点云再次变换。必须在 SCAN-Planner 自己的 `run.launch`/`advanced_param.xml` 中改成以下接口，或复制一份等价的自定义 launch：

```text
body_pose_topic=/LIO/odom_vehicle
sensor_pose_topic=/LIO/odom_imu
cloud_topic=/map_scan
grid_map/frame_id=camera_init
grid_map/cloud_is_world=true
grid_map/need_extrinsic=false
```

对应的 ROS1 launch 关键项应为：

```xml
<arg name="cloud_topic" value="/map_scan"/>
<arg name="cloud_is_world" value="true"/>
<param name="grid_map/frame_id" value="camera_init" type="str"/>
<param name="grid_map/need_extrinsic" value="false" type="bool"/>
```

`max_prediction_sec` 限制最近激光后验之后允许的纯 IMU 外推时长。超过该时长时系统停止发布快速预测，等待新的正式后验，避免把长时间纯惯导漂移继续交给规划器。SCAN-Planner 的控制循环可以高于激光雷达频率：控制器读取最近的 IMU 频率预测位姿，而每次激光观测后验会周期性校正预测起点。

Ubuntu 实机检查：

```bash
rostopic hz /LIO/odom_imu
rostopic hz /LIO/odom_vehicle
rostopic delay /LIO/odom_imu
rostopic echo -n 1 /localization/valid
rosrun tf tf_echo camera_init aft_mapped
rosrun tf tf_echo aft_mapped base_link
```

如果 `/localization/valid` 为 `false`，默认没有导航里程计输出；如果只有 `/LIO/odom_imu` 而没有 `/LIO/odom_vehicle`，应优先检查静态 TF 方向、frame 名称和适配器节流告警；如果输出在 0.5 秒后停止，应检查激光后验是否仍在更新，而不是盲目增大 `max_prediction_sec`。

### 4.6 其他类型激光雷达

其他类型激光雷达的支持将在后续发布。

## 5. VoxelSLAMPointCloud2

**VoxelSLAMPointCloud2** 是一个定制的 RViz 插件，用法与原生 `PointCloud2` 基本一致。无论插件的 **Decay Time** 设置为何，当插件收到空点云时，都可以自动清除当前显示的点云地图。

使用步骤：

1. 将 `VoxelSLAMPointCloud2` 放入 catkin 工作空间的同一个 `src` 目录，然后执行 `catkin_make`。
2. 使用 `roslaunch` 启动程序和 RViz。
3. 在 RViz 中单击 **Add**，添加 `VoxelSLAMPointCloud2` 显示项。
