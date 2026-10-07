# Voxel-SLAM地图持久化、全局重定位与只读地图定位实施交接

> **2026-10-07 高频导航接口更新：** 当前分支已经加入独立 IMU callback queue、高频 `/LIO/odom_imu`、同时间戳 `camera_init -> aft_mapped` TF，以及读取静态 `aft_mapped -> base_link` 外参并发布 `/LIO/odom_vehicle` 的 ROS1 适配节点。正式扫描批处理 IESKF、去畸变和协方差传播未被快速链替代；`LOCAL_ODOMETRY` 保留 BA，连续 `IN_MAP_TRACKING` 不使用 BA。实现提交为 `6ed5635`、`e390d27`、`e94177b`、`d69efbc`、`275795e`，最终审查修复为 `4411e78`。Windows 已通过 20/20 项 Python 契约与定位回归、8/8 份 YAML、launch XML 和 CMake 文件引用检查，但 ROS/C++ 编译、gtest、rosbag 频率与时延仍必须在 Ubuntu 20.04 + ROS Noetic 复验。完整使用说明见根目录 README 的“高频里程计与 ROS1 SCAN-Planner 接入”。

> **2026-10-06 当前状态：** 本文主体记录的是定位 v1 的设计过程，其中“按历史关键帧加载并重建局部体素地图”的接口和参数已经被后续实现取代。当前开发分支为 `codex/spatial-voxel-blocks`；最新有效设计见 `docs/superpowers/specs/2026-10-06-spatial-voxel-block-map-design.md`，实施计划见 `docs/superpowers/plans/2026-10-06-spatial-voxel-block-map.md`。当前定位连续跟踪使用离线构建的 v2 空间体素块、哈希索引、后台异步加载和不可变快照，不再使用 `rebuild_local_map()`、`update_loaded_keyframes()` 或按关键帧重建 `surf_map`。`PriorMapTracker` 仅保留 BTC 的二维候选范围查询和 RViz 初始位姿的最近高度查询。本文后文涉及旧局部关键帧缓存的内容仅作为 v1 基线与演进记录，不能作为现行接口说明。

> **For agentic workers:** REQUIRED SUB-SKILL: use `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，按本文任务顺序实施。不要在没有读完本文和所列源码的情况下直接修改主流程。

**目标：** 在现有Voxel-SLAM基础上增加一套纯LiDAR-IMU的“离线建图地图包 + 无人工初值BTC全局重定位 + GTSAM轨迹对齐 + 历史地图只读IESKF连续定位”模式。

**架构：** 建图端继续使用Voxel-SLAM的LIO、滑窗LBA、BTC回环和GTSAM优化，但新增融合关键帧、BTC描述子及验证平面云的直接持久化。定位端先在临时odom系独立运行LIO并生成同构BTC查询帧，找到历史候选并经normal ICP精修后，用固定历史锚点和当前LIO里程计边优化当前session；随后把实时前端统一校正到map系，并仅加载历史关键帧点云进行只读in-map tracking。

**技术栈：** Ubuntu 20.04、ROS Noetic、C++14、PCL 1.10、Eigen 3.3.7、GTSAM 4.0.3、Livox ROS Driver、Voxel-SLAM现有BTC/STD与LiDAR-IMU IESKF/LBA。

## 全局约束

- 只修改Voxel-SLAM，不依赖或移植Lumi-Nav代码。
- 传感器仅使用LiDAR和IMU，不引入相机。
- 使用正确名称 **BTC（Binary Triangle Combined）**，不要写成BCT。
- 历史地图在定位模式下完全只读；定位成功后不扩展、不覆盖、不合并历史地图。
- 定位丢失后保留历史地图，清理当前临时定位状态，重新进入BTC全局重定位。
- 第一版保留Voxel-SLAM现有GTSAM/iSAM2，不引入SE-Sync。
- 精配准使用 `icp_normal()`；它内部已经执行点到面高斯牛顿迭代，不再重复增加一个独立GN阶段。
- 第一版一个“满足运动阈值的多Scan融合关键帧”对应一个BTC描述子帧，不使用历史加载代码中的 `acsize/mgsize` 二次关键帧聚合。
- 建图端和定位端必须使用相同的 `win_size`、运动阈值、点云预处理、BTC参数和坐标系约定。
- 地图必须在最终回环/图优化（需要时包括GBA）结束后导出，保存最终关键帧位姿。
- 当前Windows主机没有ROS Noetic运行环境；这里可以做源码审查和纯数据格式测试设计，真正编译、rosbag和实车验证必须在Ubuntu 20.04目标机完成。
- 当前解压目录不是Git仓库（执行 `git status` 返回not a git repository）。开始代码修改前，先确认用户是否另有Git仓库副本；若没有，应初始化版本控制或复制到受控仓库，禁止无备份地大改唯一源码。

---

## 1. 用户已经确认的产品行为

### 1.1 建图模式

```text
LiDAR + IMU
  → Voxel-SLAM初始化、IESKF里程计、滑窗LBA
  → 满足运动阈值后构造多Scan融合关键帧
  → 为该融合子地图生成BTC描述子帧和验证平面云
  → 在线BTC回环 + normal ICP验证
  → GTSAM/iSAM2优化（结束时可运行原有GBA）
  → 导出只读地图包
```

### 1.2 定位模式

```text
加载只读地图包和历史BTC数据库
  → 新session在临时odom系运行LIO
  → 构造与建图端一致的多Scan融合BTC查询帧
  → BTC全局检索历史候选及初始相对位姿
  → normal ICP（内部GN）几何验证和六自由度精修
  → 固定历史锚点，优化当前session关键帧轨迹
  → 计算 T_map_odom 并校正实时状态/滑窗/轨迹
  → 根据当前位置加载历史关键帧点云
  → 每个Scan执行IMU传播 + 历史地图点到面IESKF更新
  → 定位质量连续不合格则进入LOST并重新BTC重定位
```

### 1.3 非目标

- 第一版不做视觉重定位。
- 第一版不做在线地图更新、长期地图维护或多地图融合。
- 第一版不实现Lumi-Nav的平面Patch数据库和BnB。
- 第一版不替换GTSAM为SE-Sync。
- 第一版不做云端地图服务、TCP传输或GUI。
- 第一版不追求把原版所有历史逐Scan数据兼容成新地图包；可以另写一次性转换工具，但不能污染运行时主流程。

---

## 2. 必须先理解的现有源码事实

仓库根目录：

```text
Voxel-SLAM-main/Voxel-SLAM-main/
```

ROS包目录：

```text
VoxelSLAM/
```

### 2.1 关键源码

- `VoxelSLAM/src/voxelslam.cpp`
  - `FileReaderWriter::save_pcd()`、`save_pose()`：原版逐Scan地图保存。
  - `FileReaderWriter::previous_map_read()`：读取历史逐Scan PCD/位姿，重建融合关键帧和BTC库。
  - `VOXEL_SLAM::loop_update()`：图优化成功后的前端状态、滑窗和局部图校正。
  - `VOXEL_SLAM::keyframe_loading()`：现有关键帧空间加载逻辑，但当前主要面向当前session旧关键帧，不等价于固定历史地图定位。
  - `VOXEL_SLAM::thd_loop_closure()`：多Scan融合关键帧、BTC查询、跨session回环、iSAM2优化和 `map_loop` 重建。
- `VoxelSLAM/src/BTC.h`、`VoxelSLAM/src/BTC.cpp`
  - `STD`、`BinaryDescriptor`、`STDescManager`。
  - `GenerateSTDescs()`：生成描述子并向 `plane_cloud_vec_` 添加该描述子帧的验证平面云。
  - `SearchLoop()`：哈希检索、候选投票、三角形求相对位姿及平面几何验证。
  - `AddSTDescs()`：将一帧描述子加入哈希数据库，并递增 `current_frame_id_`。
- `VoxelSLAM/src/loop_refine.hpp`
  - `ScanPose`。
  - `icp_normal()`：点到面残差的20轮GN精修及法向可观测性检查。
  - GTSAM `BetweenFactor<Pose3>` 边构造与多session `PGO_Edges`。
- `VoxelSLAM/src/voxel_map.hpp`
  - `Keyframe`：`x0`、局部系降采样点云 `plptr`、加载标记等。
  - `down_sampling_pvec()`。
- `VoxelSLAM/CMakeLists.txt`
  - 当前只编译 `src/voxelslam.cpp` 和 `src/BTC.cpp`，新增 `.cpp` 必须显式加入目标或拆成库。

### 2.2 现有关键帧与BTC的真实数据流

在线回环线程从LBA退窗Scan中维护 `bl_local`。当数量达到 `win_size` 且相对上个关键位姿满足运动阈值时，以最新Scan状态 `xc` 为参考，把窗口内第i帧点变换到最新Scan坐标系：

```text
p_current = R_current^T * (R_i * p_i + t_i - t_current)
```

融合结果是 `pptr`：

```text
pptr ──down_sampling_pvec(voxel_size/10)──> Keyframe::plptr
  │
  └──直接转换（未使用plptr）──────────────> plbtc -> GenerateSTDescs()
```

因此：

- 关键帧位姿是窗口最新Scan的位姿。
- `Keyframe::plptr` 是融合点云的降采样版本，表达在关键帧IMU/body坐标系。
- BTC与关键帧点云同源，但BTC输入是未经过该关键帧降采样的融合 `pptr`。
- 运动不足时窗口逐Scan滑动，不是每固定 `win_size` 帧无条件生成关键帧。
- `GenerateSTDescs()`中的 `STD.frame_number_` 使用 `STDescManager::current_frame_id_`（连续描述子帧编号），不是传入的Scan ID。
- `GenerateSTDescs(..., id)`把传入的 `id` 写入 `plane_cloud->header.seq`；原版通过它从描述子帧反查历史Scan/关键帧锚点。

### 2.3 原版落盘行为

原版主要保存逐Scan PCD、`alidarState.txt`和图边；历史加载时：

1. 每 `win_size` 个Scan重建一个融合关键帧；
2. 再按 `acsize=10`、`mgsize=5`聚合多个关键帧；
3. 重新生成BTC库。

新方案直接保存融合关键帧和BTC帧，加载时不再执行上述二次聚合，否则建图端和定位端描述子尺度不一致。

### 2.4 原版已经有但不能直接当成定位器的能力

原版能加载历史session、重建BTC、检测跨session回环、iSAM2优化、计算 `dx` 并调用 `loop_update()`。但是回环后的 `map_loop` 和 `pl_kdmap` 主要由当前session关键帧构建，系统继续SLAM；它没有完整的只读历史地图层、TRACKING/LOST状态和历史关键帧按需加载生命周期。

---

## 3. 新地图包格式（版本1）

建议每张地图是独立目录：

```text
map_name/
├── manifest.yaml
├── keyframes.csv
├── keyframes/
│   ├── 000000.pcd
│   ├── 000001.pcd
│   └── ...
├── btc_frames.bin
├── btc_planes/
│   ├── 000000.pcd
│   ├── 000001.pcd
│   └── ...
└── pose_graph_edges.txt        # 可选，定位第一版不依赖
```

### 3.1 `manifest.yaml`

必须保存并在加载时校验：

```yaml
format: voxel_slam_map
version: 1
coordinate_frame: map
keyframe_cloud_frame: imu
win_size: 10
keyframe_distance_m: 0.1
keyframe_angle_deg: 5.0
keyframe_voxel_size: 0.1
descriptor_policy: one_per_fused_keyframe
btc_config_fingerprint: "<由全部BTC参数稳定序列化后计算的摘要>"
keyframe_count: 0
btc_frame_count: 0
```

加载时版本、坐标系、描述子策略和BTC参数摘要不匹配必须失败，不能静默继续。

### 3.2 `keyframes.csv`

每行：

```text
kf_id,timestamp,tx,ty,tz,qx,qy,qz,qw,cloud_file
```

- 位姿必须是最终优化后的 `T_map_keyframe`。
- 点云必须表达在关键帧IMU/body局部坐标系，禁止提前变换到map系后再保存。
- `kf_id`在地图内连续，从0开始，便于KD-Tree和数组索引。

### 3.3 `btc_frames.bin`

不要直接序列化 `unordered_map<STD_LOC,...>`，其布局不稳定。按描述子帧顺序保存原始 `vector<STD>`，加载后依次调用 `AddSTDescs()`重建哈希表。

文件头：

```cpp
struct BtcFileHeader {
  char magic[8];          // "VXLBTC01"
  uint32_t version;       // 1
  uint32_t frame_count;
};
```

每帧记录：

```cpp
struct BtcFrameHeader {
  uint32_t frame_id;            // 必须等于0..frame_count-1
  uint32_t anchor_keyframe_id;  // 对应关键帧
  uint32_t descriptor_count;
};
```

每个 `STD` 保存：

- `triangle_`、`angle_`、`center_`（各3个double）；
- `frame_number_`（加载时强制改写为当前 `frame_id`）；
- 三个 `BinaryDescriptor`；
- 每个BinaryDescriptor保存 `summary_`、`location_`、位数组长度和按bit打包后的 `occupy_array_`。

`btc_planes/<frame_id>.pcd`保存与该BTC帧同坐标系的 `PointXYZINormal`，用于 `SearchLoop()`内的平面几何验证和后续 `icp_normal()`。

### 3.4 数据一致性要求

- `btc_frame_id`必须与 `STDescManager::plane_cloud_vec_`下标一致。
- `anchor_keyframe_id`不能依赖PCL `header.seq`作为唯一持久化字段；新格式应显式保存映射，加载后可同时恢复 `header.seq`以兼容旧调用链。
- 关键帧点云、BTC顶点位置、BTC验证平面云都必须表达在各自anchor关键帧局部坐标系。
- 保存过程中使用临时目录，全部文件成功后再原子重命名为最终地图目录，防止半张地图被加载。

---

## 4. 建议新增的代码边界

避免继续把所有逻辑堆进 `voxelslam.cpp`。

### 4.1 地图数据和IO

创建：

```text
VoxelSLAM/src/map_package.hpp
VoxelSLAM/src/map_package.cpp
```

建议接口：

```cpp
struct SavedPose {
  double stamp = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct SavedKeyframe {
  int id = -1;
  SavedPose pose;
  pcl::PointCloud<PointType>::Ptr cloud;
};

struct SavedBtcFrame {
  int frame_id = -1;
  int anchor_keyframe_id = -1;
  std::vector<STD> descriptors;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud;
};

struct SavedMap {
  std::vector<SavedKeyframe> keyframes;
  std::vector<SavedBtcFrame> btc_frames;
};

class MapPackageIO {
public:
  static bool save(const std::string &dir, const SavedMap &map,
                   const ConfigSetting &btc_config, std::string &error);
  static bool load(const std::string &dir, SavedMap &map,
                   ConfigSetting &btc_config, std::string &error);
};
```

### 4.2 BTC持久化辅助

创建：

```text
VoxelSLAM/src/btc_io.hpp
VoxelSLAM/src/btc_io.cpp
```

建议接口：

```cpp
bool save_btc_frames(const std::string &path,
                     const std::vector<SavedBtcFrame> &frames,
                     std::string &error);
bool load_btc_frames(const std::string &path,
                     std::vector<SavedBtcFrame> &frames,
                     std::string &error);
bool restore_btc_database(const std::vector<SavedBtcFrame> &frames,
                          STDescManager &manager,
                          std::string &error);
```

`restore_btc_database()`必须按frame_id升序执行：

```text
校验frame_id连续
→ 将本帧所有STD.frame_number_重写为frame_id
→ plane_cloud_vec_.push_back(plane_cloud)
→ AddSTDescs(descriptors)
```

### 4.3 定位状态机

创建：

```text
VoxelSLAM/src/localization_state.hpp
```

状态固定为：

```cpp
enum class LocalizationState {
  WAITING_FOR_MAP,
  LOCAL_ODOMETRY,
  RELOCALIZING,
  ALIGNING,
  IN_MAP_TRACKING,
  LOST
};
```

合法迁移：

```text
WAITING_FOR_MAP -> LOCAL_ODOMETRY       地图加载成功
LOCAL_ODOMETRY -> RELOCALIZING         形成有效BTC查询帧
RELOCALIZING -> ALIGNING               BTC+ICP通过
RELOCALIZING -> LOCAL_ODOMETRY         本次候选失败，继续积累
ALIGNING -> IN_MAP_TRACKING            图优化和前端交接完成
IN_MAP_TRACKING -> LOST                连续质量失败
LOST -> LOCAL_ODOMETRY                 清理临时session并重新初始化
```

禁止 `IN_MAP_TRACKING` 更新历史地图文件或历史关键帧对象。

### 4.4 历史地图管理与跟踪

创建：

```text
VoxelSLAM/src/prior_map_tracker.hpp
VoxelSLAM/src/prior_map_tracker.cpp
```

建议接口：

```cpp
struct TrackingQuality {
  int matched_points = 0;
  double mean_abs_point_plane = 0.0;
  double min_normal_eigenvalue = 0.0;
  bool finite = true;
};

class PriorMapTracker {
public:
  bool initialize(const SavedMap &map, std::string &error);
  void reset_runtime_cache();
  bool rebuild_local_map(const Eigen::Vector3d &position,
                         double radius,
                         int preload_limit);
  bool update_loaded_keyframes(const Eigen::Vector3d &position,
                               double load_radius,
                               double unload_radius,
                               int max_load_per_scan);
  bool tracking_is_lost(const TrackingQuality &quality);
};
```

初始建议参数写入配置文件并可调：

```yaml
Localization:
  map_path: ""
  local_map_radius: 20.0
  unload_radius: 30.0
  preload_keyframes: 40
  max_load_per_scan: 1
  min_matched_points: 30
  max_mean_abs_point_plane: 0.30
  min_normal_eigenvalue: 1.0
  lost_streak: 5
```

这些是保守起始值，不是论文结论；目标机实验后再标定，但代码必须有明确默认值和日志。

---

## 5. 坐标系和相对位姿约定

必须在代码注释和日志中统一：

- `W`：历史地图map坐标系；
- `O`：当前session重定位前的临时odom坐标系；
- `H`：命中的历史BTC anchor关键帧；
- `C`：当前BTC查询anchor关键帧。

`SearchLoop()`/`triangle_solver()`当前匹配对中，第一项来自当前查询，第二项来自历史数据库，输出应按源码和离线单测确认成：

```text
p_H = R_HC * p_C + t_HC
```

即 `T_H_C`。不要仅凭变量名猜方向，必须用合成点云单测确认。

由历史位姿 `T_W_H`得到当前地图位姿：

```text
T_W_C = T_W_H * T_H_C
```

优化后用当前anchor新旧位姿计算统一校正：

```text
T_W_O = T_W_C_optimized * inverse(T_O_C_old)
```

并原子应用到：

- `x_curr.R/p/v`；
- `x_buf`全部滑窗状态；
- 尚未入轨迹的缓存ScanPose；
- 当前session轨迹；
- 当前临时局部点云。

重力只应按姿态校正旋转，不做平移。协方差应按误差状态定义进行旋转变换，不能简单保持不变而不说明。

---

## 6. 图优化设计

第一版不需要把整张历史位姿图全部放进在线优化器。建立一个“固定历史锚点 + 当前session轨迹”的小图：

```text
历史anchor H：PriorFactor，使用强约束固定在T_W_H
当前节点 C0...Cn：由LIO相对里程计边连接
H <-> Ck：BTC+normal ICP得到的跨session边
```

关键要求：

- 删除或弱化当前session第一节点在临时odom原点上的强Prior；否则它会与跨session边冲突，无法把当前轨迹拉到历史地图。
- 历史地图是唯一Gauge。
- 至少积累两个当前关键帧再执行图优化，以验证里程计边方向；只有一个当前节点时图优化退化为一次刚体对齐，可以直接计算但仍应走统一接口。
- 第一版接受一次高置信跨session约束即可进入ALIGNING；后续可以增加连续多个BTC帧一致性，但不应阻塞MVP。
- 图优化失败或结果出现NaN/Inf、位移超过可配置安全上限时，不允许进入IN_MAP_TRACKING。

建议新增独立函数，而不是把逻辑继续写在 `thd_loop_closure()`大函数中：

```cpp
bool align_current_session_to_map(
    const SavedPose &history_anchor,
    int current_anchor_index,
    const Eigen::Matrix3d &R_HC,
    const Eigen::Vector3d &t_HC,
    const std::vector<ScanPose*> &current_poses,
    std::vector<gtsam::Pose3> &optimized,
    IMUST &T_W_O,
    std::string &error);
```

---

## 7. 只读in-map tracking设计

### 7.1 历史关键帧索引

地图加载后，用所有 `T_W_K.t`建立PCL KD-Tree。点的 `curvature`或并行数组保存 `kf_id`；禁止依赖KD-Tree返回下标恰好等于关键帧ID，必须显式映射。

### 7.2 局部参考地图

进入IN_MAP_TRACKING时：

1. 清空重定位前的临时 `surf_map`和滑窗地图层；
2. 在优化后的当前位置附近预加载最多 `preload_keyframes` 个历史关键帧；
3. 用最终 `T_W_K`把局部关键帧云变换到map系；
4. 以固定点形式写入定位参考体素图；
5. `recut`/平面拟合完成后才允许前端继续更新。

每个后续Scan：

```text
IMU传播和去畸变
→ 当前点与固定历史surf_map建立点到面残差
→ IESKF迭代更新
→ 最多加载1帧新进入半径的历史KF
→ 释放超过unload_radius且不在当前匹配使用中的历史KF体素
→ 输出TrackingQuality
```

### 7.3 只读保证

- 在IN_MAP_TRACKING中，禁止把当前Scan永久插入历史参考层。
- 如为短时匹配必须维护当前Scan缓存，应放在独立临时层，状态退出时全部清空。
- 禁止写回 `SavedMap`、历史PCD和BTC文件。
- 禁止运行会修改历史关键帧位姿的GBA。
- ROS节点退出时，定位模式不调用地图导出。

### 7.4 丢失判定

满足任一硬故障立即LOST：

- 状态或残差出现NaN/Inf；
- 当前位姿附近没有任何历史关键帧；
- IESKF线性系统求解失败。

软故障连续 `lost_streak` 帧触发LOST：

- `matched_points < min_matched_points`；
- `mean_abs_point_plane > max_mean_abs_point_plane`；
- `min_normal_eigenvalue < min_normal_eigenvalue`。

LOST处理：

```text
停止发布“定位有效”标志
→ 清空临时当前session滑窗、BTC查询缓存和临时地图层
→ 保留只读SavedMap、BTC数据库和历史KD-Tree
→ 重新初始化局部LIO
→ 回到LOCAL_ODOMETRY
```

---

## 8. 分阶段实施计划

### Task 1：建立可测试的地图数据模型与构建目标

**文件：**

- 创建 `VoxelSLAM/src/map_package.hpp`
- 创建 `VoxelSLAM/src/map_package.cpp`
- 创建 `VoxelSLAM/test/map_package_roundtrip_test.cpp`
- 修改 `VoxelSLAM/CMakeLists.txt`
- 修改 `VoxelSLAM/package.xml`（仅在确实增加测试依赖时）

**产出：** `SavedMap`、关键帧CSV/PCD读写和明确错误消息。

**测试：** 构造两个局部点云和已知位姿，保存后读取，逐字段比较；损坏版本号、缺失PCD和非连续ID必须失败。

**Ubuntu命令：**

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
```

预期：构建成功，round-trip测试通过，故障输入测试返回预期错误而不是崩溃。

### Task 2：实现BTC数据库序列化和恢复

**文件：**

- 创建 `VoxelSLAM/src/btc_io.hpp`
- 创建 `VoxelSLAM/src/btc_io.cpp`
- 创建 `VoxelSLAM/test/btc_io_roundtrip_test.cpp`
- 修改 `VoxelSLAM/CMakeLists.txt`

**产出：** `vector<SavedBtcFrame>`二进制读写，以及恢复 `STDescManager`的函数。

**测试：** 至少保存两帧、每帧多个STD和不同长度的 `occupy_array_`；读取后验证全部数值、bit位、frame ID和plane cloud数量；再用同一查询STD比较保存前后 `SearchLoop()`候选一致性。

### Task 3：在建图流程中采集并导出融合关键帧/BTC帧

**文件：**

- 修改 `VoxelSLAM/src/voxelslam.cpp` 中关键帧生成和程序收尾路径
- 修改 `VoxelSLAM/config/*.yaml`，增加 `General/mode`、`General/map_export_path`

**产出：** 建图模式运行期间保存内存记录，最终优化完成后调用 `MapPackageIO::save()`。

**关键实现：**

- `smp->plptr`复制为 `SavedKeyframe.cloud`；
- 关键帧初建时记录anchor Scan ID和时间戳；
- `GenerateSTDescs()`后立即复制 `stds_vec`及 `plane_cloud_vec_.back()`为 `SavedBtcFrame`；
- 最终导出前使用优化后的关键帧位姿覆盖 `SavedKeyframe.pose`；
- `frame_id`使用连续BTC帧序号，`anchor_keyframe_id`使用显式映射；
- 任何一个文件保存失败则整个导出失败，不留下可被加载的最终目录。

### Task 4：加载只读地图并恢复历史BTC数据库

**文件：**

- 创建 `VoxelSLAM/src/prior_map_tracker.hpp`
- 创建 `VoxelSLAM/src/prior_map_tracker.cpp`
- 修改 `VoxelSLAM/src/voxelslam.cpp` 初始化路径
- 创建 `VoxelSLAM/test/prior_map_index_test.cpp`

**产出：** 定位模式启动时加载地图、恢复BTC、建立历史关键帧KD-Tree，状态从WAITING_FOR_MAP进入LOCAL_ODOMETRY。

**测试：** 在已知关键帧位置集合中做半径查询，验证返回显式kf_id；参数摘要不一致时启动失败。

### Task 5：实现无初值BTC重定位

**文件：**

- 创建 `VoxelSLAM/src/relocalizer.hpp`
- 创建 `VoxelSLAM/src/relocalizer.cpp`
- 修改 `VoxelSLAM/src/voxelslam.cpp`，把当前关键帧构造复用为查询帧
- 创建 `VoxelSLAM/test/relocalizer_transform_test.cpp`

**产出：** 当前融合子地图BTC查询、候选历史anchor、`T_H_C`和质量指标。

**测试：** 用合成点云施加已知刚体变换，验证 `SearchLoop()+icp_normal()`输出方向和误差；至少覆盖90° yaw、非零xyz和平面退化失败用例。

### Task 6：实现固定历史锚点的当前session图优化

**文件：**

- 创建 `VoxelSLAM/src/session_aligner.hpp`
- 创建 `VoxelSLAM/src/session_aligner.cpp`
- 创建 `VoxelSLAM/test/session_aligner_test.cpp`
- 修改 `VoxelSLAM/CMakeLists.txt`

**产出：** `align_current_session_to_map()`，输出所有当前节点优化位姿和 `T_W_O`。

**测试：** 构造有已知漂移的5节点轨迹、一个固定历史anchor和一条跨session边；验证历史anchor不动、当前轨迹整体进入map系、相邻里程计结构保持一致。额外测试错误变换方向应无法通过误差断言。

### Task 7：复用并收紧前端交接

**文件：**

- 修改 `VoxelSLAM/src/voxelslam.cpp::loop_update()`，拆出可复用的 `apply_global_correction()`
- 创建 `VoxelSLAM/test/global_correction_test.cpp`

**产出：** 对当前状态、速度、重力、滑窗、轨迹和缓存点云的一致原子校正；校正成功后状态进入ALIGNING末尾。

**测试：** 对已知点、姿态和速度应用固定 `T_W_O`，验证公式及两次重复调用保护，防止同一校正量应用两次。

### Task 8：实现只读历史局部地图和in-map IESKF

**文件：**

- 完成 `VoxelSLAM/src/prior_map_tracker.cpp`
- 修改 `VoxelSLAM/src/voxelslam.cpp` 的主LIO循环和地图更新分支
- 创建 `VoxelSLAM/test/prior_map_loading_test.cpp`

**产出：** 进入IN_MAP_TRACKING后清除临时地图，预加载历史关键帧，随后每Scan动态加载/释放；当前Scan不进入固定历史层。

**测试：** 构造三组相距较远的关键帧，模拟机器人移动，验证加载集合按半径变化、同一关键帧不重复加载、越界帧释放、`SavedMap`内容哈希前后不变。

### Task 9：实现质量监控、LOST和自动重定位

**文件：**

- 完成 `VoxelSLAM/src/localization_state.hpp`
- 修改 `VoxelSLAM/src/voxelslam.cpp` 的状态分发
- 增加ROS状态发布（建议 `std_msgs/String` 和 `std_msgs/Bool`）
- 创建状态机单元测试

**产出：** 连续质量失败进入LOST，清理当前临时状态但保留历史地图，再自动返回LOCAL_ODOMETRY。

**测试：** 输入质量序列GOOD→5次BAD→GOOD，验证状态迁移、有效标志和缓存清理次数；NaN应单帧直接LOST。

### Task 10：配置、文档和端到端验证

**文件：**

- 修改 `VoxelSLAM/README.md`或仓库根 `README.md`
- 新增 `VoxelSLAM/config/localization_mid360.yaml`
- 新增 `VoxelSLAM/launch/vxlm_localization_mid360.launch`
- 新增离线评测脚本目录（建议 `VoxelSLAM/scripts/evaluate_localization.py`，仅评测，不参与运行时）

**端到端验证：**

1. 同一数据第一次运行建图并导出地图包；
2. 第二次从不同起点/朝向运行定位，不提供人工初始位姿；
3. 记录首次有效定位耗时、BTC候选得分、ICP内点/特征值；
4. 对比定位轨迹与真值或建图轨迹，计算ATE/RPE；
5. 重放无重叠场景，验证不会误报定位成功；
6. 人为遮挡或切断有效几何，验证进入LOST并能在再次进入已知区域后恢复；
7. 定位前后计算地图目录文件哈希，验证只读不变。

---

## 9. 关键风险与禁止事项

### 9.1 BTC帧编号与Scan/KF编号混用

`STD.frame_number_`是连续描述子帧编号；`plane_cloud.header.seq`原版保存传入Scan ID。新代码必须显式维护 `btc_frame_id -> anchor_keyframe_id`，禁止用一个整数同时表达两种编号。

### 9.2 变换方向写反

BTC、ICP、GTSAM BetweenFactor和 `dx`使用的方向不同。所有接口名必须带源/目标语义，并用合成点云测试，不允许只看变量名推断。

### 9.3 当前session原点Prior冲突

重定位前临时odom原点不能在对齐图中继续作为强Prior。历史anchor才是唯一Gauge。

### 9.4 描述子输入不一致

地图端若使用多关键帧二次聚合，定位端却只用一个融合关键帧，召回率会不可控。第一版统一为“一融合关键帧一BTC帧”。

### 9.5 当前点污染历史地图

不能简单复用SLAM模式的 `cut_voxel_multi()`持续写入同一个 `surf_map`。必须区分固定历史层和临时当前层，或在IN_MAP_TRACKING显式跳过当前Scan地图插入。

### 9.6 只做BTC不做几何验证

BTC候选可能误匹配。必须通过 `SearchLoop()`现有平面验证、`icp_normal()`、可观测性阈值和最终位姿合理性门限后才能进入ALIGNING。

### 9.7 在Windows上声称验证通过

当前主机不能构建ROS Noetic工程。没有Ubuntu编译日志、单元测试结果和rosbag结果时，只能写“完成源码修改/静态核对”，不能写“运行通过”。

---

## 10. 新会话开始工作的建议顺序

1. 完整阅读本文件。
2. 阅读仓库根README及第2.1节列出的源码函数。
3. 再次执行 `git status --short --branch`，确认实际工作副本是否受Git管理；若仍不是Git仓库，先向用户确认目标仓库位置或版本控制策略。
4. 不要直接改 `thd_loop_closure()`；先完成Task 1和Task 2的独立IO与测试。
5. 每完成一个Task，提供：变更文件、测试命令、实际输出、尚未验证项。
6. 目标机首次集成前，先使用合成点云验证BTC/ICP变换方向，再接rosbag。
7. 第一版完成标准是：无人工初值进入历史地图、连续IESKF定位、历史地图文件哈希不变、丢失后自动重定位。

---

## 11. 简历职责边界（供后续整理，不是代码需求）

只有在上述功能真实实现并验证后，才可以表述为：

> 基于Voxel-SLAM设计建图—定位双模式，扩展融合关键帧与BTC地图持久化，通过BTC全局检索、点到面ICP和GTSAM完成跨Session重定位及轨迹对齐，并基于历史关键帧动态加载和LiDAR-IMU IESKF实现只读地图连续定位。

不要把原版Voxel-SLAM的LIO、BTC算法或GTSAM框架写成自己原创；自己的贡献应聚焦于地图格式、BTC持久化、定位状态机、历史地图只读跟踪、前端交接及丢失恢复。

