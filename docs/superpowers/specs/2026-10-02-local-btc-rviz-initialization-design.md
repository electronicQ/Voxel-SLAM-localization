# Voxel-SLAM 局部 BTC 与 RViz 直接初始化设计

## 1. 背景与目标

本文是 `HANDOFF.md` 中“无人工初值 BTC 全局重定位”方案的增量设计。系统仍以 BTC（Binary Triangle Combined）作为自动重定位方法，同时增加两项能力：

1. 可通过 YAML 中的二维先验位置限制历史 BTC 候选范围，并在失败时逐级扩大范围，最后按配置回退到全局 BTC。
2. 可随时接收 RViz 的 `/initialpose`。该路径跳过 BTC，以人工给定的二维位姿直接构建历史局部地图，经过点到面匹配验证后才进入有效定位。

本设计不增加“不支持 BTC”的构建或运行模式。建图和自动重定位均假设 BTC 模块可用。

## 2. 已确认的产品行为

### 2.1 自动 BTC 初始化

- `use_prior_pose_for_btc: true`：读取 YAML 中的 `x/y`，只允许该位置附近的历史关键帧对应的 BTC 帧参与匹配。
- `use_prior_pose_for_btc: false`：忽略先验位置，直接执行全地图 BTC。
- BTC 范围先验只包含 `x/y`，不使用 yaw 约束。
- 局部 BTC 失败时逐级扩大二维搜索半径；达到最大半径后，根据 `btc_fallback_to_global` 决定是否执行全局 BTC。
- BTC 候选仍必须通过现有平面几何验证、`icp_normal()`、可观测性检查和位姿安全门限。

### 2.2 RViz 直接初始化

- 订阅 ROS 标准话题 `/initialpose`，消息类型为 `geometry_msgs/PoseWithCovarianceStamped`。
- 只接受 `header.frame_id == "map"` 的请求，不隐式猜测或转换其他坐标系。
- `x/y/yaw` 来自 RViz；`roll/pitch` 来自当前已完成初始化的 IMU；初始 `z` 取二维距离最近的历史关键帧高度。
- RViz 路径不执行 BTC，直接加载附近历史关键帧并构建只读局部地图。
- RViz 位姿只是初始化候选，不等于定位成功。必须经过局部点到面精配准和连续多帧质量验证，才进入有效定位。
- RViz 请求在任何定位状态下都具有最高优先级。新请求覆盖旧请求，并中止尚未提交的 BTC 或旧 RViz 初始化结果。
- RViz 验证失败后清除本次临时状态，并恢复 YAML 配置指定的 BTC 重定位流程。

## 3. 配置

```yaml
Localization:
  use_prior_pose_for_btc: true

  # 仅在use_prior_pose_for_btc=true时读取。
  btc_prior_position:
    x: 0.0
    y: 0.0

  btc_search_radius: 30.0
  btc_expand_on_failure: true
  btc_max_search_radius: 100.0
  btc_fallback_to_global: true

  rviz_initialpose_enabled: true
  direct_pose_validation_scans: 3
  direct_pose_validation_timeout_sec: 5.0
```

约束如下：

- `btc_search_radius`必须大于0。
- `btc_max_search_radius`必须不小于`btc_search_radius`。
- `direct_pose_validation_scans`必须大于0。
- `direct_pose_validation_timeout_sec`必须大于0。
- `use_prior_pose_for_btc=true`时，`btc_prior_position/x`和`y`必须存在且为有限数值。
- `use_prior_pose_for_btc=false`时，可以不写`btc_prior_position`，加载器不得要求该字段存在。
- 配置非法时定位模式启动失败并给出明确日志，不静默替换用户值。

第一版半径扩展采用确定性倍增：

```text
r_next = min(2 * r_current, btc_max_search_radius)
```

例如初始半径30 m、最大半径100 m时，搜索序列为30、60、100 m。最大半径失败后再决定是否全局搜索。

## 4. 历史关键帧索引

地图包加载成功后建立两个只读空间索引：

1. 二维 XY 索引：将每个最终关键帧位置写为 `(x, y, 0)`。用于 BTC 范围查询和 RViz 初始化时查找最近关键帧高度。
2. 三维位置索引：保存最终 `(x, y, z)`。用于 `IN_MAP_TRACKING` 中历史关键帧的动态加载和释放。

两个索引都必须通过并行数组或点字段显式保存“索引点到 `keyframe_id`”的映射，禁止假设 KD-Tree 返回下标等于关键帧 ID。

地图格式版本1中，一个融合关键帧对应一个 BTC 帧，但运行时代码仍通过地图包中显式保存的 `btc_frame_id -> anchor_keyframe_id` 映射进行转换，不依赖 PCL `header.seq` 作为唯一关系。

## 5. 局部 BTC 候选过滤

### 5.1 接口

保留全局搜索接口，并为范围搜索增加允许帧集合：

```cpp
void SearchLoop(/* existing arguments */);

void SearchLoop(
    /* existing arguments */,
    const std::unordered_set<int> &allowed_frame_ids);
```

`candidate_selector()`提供对应重载或内部可选过滤参数。没有过滤集合表示全局搜索；显式传入空集合表示本轮没有候选，必须返回失败而不是退化为全局搜索。

### 5.2 过滤位置

过滤发生在描述子投票阶段。在访问历史描述子的 `frame_number_` 后、写入 `match_array`和候选匹配列表前，检查该帧是否属于 `allowed_frame_ids`。范围外描述子不得贡献票数，也不得进入几何验证。

不能先运行全局 `SearchLoop()`再丢弃范围外最佳结果，因为全局高票错误候选可能遮蔽范围内正确候选。

定位查询属于跨 Session 匹配。其候选有效性不能依赖原版用于同 Session 回环的 `skip_near_num_`时间邻近判断；实现时应显式区分跨 Session 查询，防止当前查询帧编号与历史帧编号的差值错误地过滤全部候选。

### 5.3 范围搜索流程

```text
YAML先验x/y
→ 二维KD-Tree半径查询
→ 得到anchor_keyframe_id集合
→ 显式映射为allowed btc_frame_id集合
→ 当前融合关键帧生成BTC描述子和验证平面云
→ 仅对allowed集合投票
→ SearchLoop平面几何验证
→ icp_normal精配准
→ GTSAM当前Session对齐
```

当前半径内没有关键帧、允许集合为空或 BTC/ICP 失败，都视为本级半径失败。若允许扩展，则查询下一级半径；否则按配置决定全局回退。

## 6. RViz 直接位姿初始化

### 6.1 消息接收与线程边界

ROS 回调只完成以下工作：

- 检查功能开关、坐标系和有限数值；
- 提取 `x/y/yaw`；
- 写入带单调递增请求序号的待处理缓存。

回调线程不得构建局部地图、执行 ICP 或修改 IESKF 状态。定位主线程消费最新请求；耗时步骤之间检查请求序号，如果出现更新请求，旧结果立即作废。

### 6.2 六自由度初值

构造候选当前位姿 `T_W_C_seed`：

- 平移 `x/y`：RViz；
- 平移 `z`：二维距离最近的历史关键帧最终高度；
- 姿态 `yaw`：RViz；
- 姿态 `roll/pitch`：当前 IMU 重力对齐结果。

IMU 尚未初始化、姿态非有限或给定 XY 附近不存在历史关键帧时，本次 RViz 请求失败，不使用零姿态冒充有效 IMU 结果。

### 6.3 局部地图与验证

1. 使用二维位置查询附近历史关键帧。
2. 用历史最终位姿把关键帧局部点云变换到 map 系。
3. 构建只读局部参考层并完成体素重切分/平面拟合。
4. 使用当前多 Scan 融合关键帧进行点到面精配准，在副本上细化候选六自由度位姿。
5. 精配准通过后，由当前 anchor 的旧临时 odom 位姿计算：

```text
T_W_O = T_W_C_refined * inverse(T_O_C_old)
```

6. 复用 `apply_global_correction()`，一次性校正当前状态、滑窗、轨迹和临时点云。
7. 保持 `localization_valid=false`，继续执行历史地图 IESKF。
8. 连续 `direct_pose_validation_scans` 帧质量合格后进入 `IN_MAP_TRACKING`。

验证失败后不尝试把同一 RViz 请求反复应用。系统清除本次临时局部状态，重新初始化当前局部 LIO，然后恢复 BTC 重定位。

## 7. 状态机与优先级

沿用已有状态，并增加对齐来源：

```cpp
enum class AlignmentSource {
  BTC,
  RVIZ_INITIAL_POSE
};
```

状态迁移：

```text
WAITING_FOR_MAP
  → 地图加载成功 → LOCAL_ODOMETRY

LOCAL_ODOMETRY
  → 形成融合查询关键帧 → RELOCALIZING

RELOCALIZING
  → BTC+ICP成功 → ALIGNING(source=BTC)
  → 收到RViz请求 → ALIGNING(source=RVIZ_INITIAL_POSE)

ALIGNING(source=BTC)
  → 图优化和前端交接成功 → IN_MAP_TRACKING
  → 失败 → LOCAL_ODOMETRY

ALIGNING(source=RVIZ_INITIAL_POSE)
  → 局部验证连续合格 → IN_MAP_TRACKING
  → 失败或超时 → LOCAL_ODOMETRY并恢复BTC

IN_MAP_TRACKING
  → 收到RViz请求 → 撤销有效标志并重新ALIGNING
  → 连续质量失败 → LOST

LOST
  → 清理当前Session临时状态并保留历史地图
  → LOCAL_ODOMETRY
  → 自动恢复配置指定的BTC路径
```

RViz 请求优先于尚未提交的 BTC 结果。任何异步 BTC/ICP/GTSAM 结果在提交前都必须检查请求序号和当前状态，防止过期结果覆盖人工初始化。

## 8. 模块边界

### `BTC.h/.cpp`

- 增加允许 BTC 帧集合的搜索重载。
- 在 `candidate_selector()`投票阶段过滤。
- 保持无过滤时的原有全局搜索行为。

### `relocalizer.hpp/.cpp`

- 执行二维范围查询和关键帧到 BTC 帧映射。
- 管理半径扩展与全局回退。
- 调用 BTC、平面验证和 `icp_normal()`。
- 返回结果和失败原因，不直接修改前端状态。

### `prior_map_tracker.hpp/.cpp`

- 建立二维、三维关键帧索引。
- 提供二维半径查询和最近关键帧高度查询。
- 构建只读局部历史地图。
- 输出逐 Scan `TrackingQuality`。

### `localization_state.hpp`

- 保存定位状态和 `AlignmentSource`。
- 定义带序号的待处理 RViz 请求数据。

### `voxelslam.cpp`

- 读取配置并订阅 `/initialpose`。
- 在主定位线程中分发状态。
- 复用统一的全局校正和前端交接函数。
- 发布定位状态和有效标志。

## 9. 质量门限与错误处理

直接位姿验证沿用只读地图跟踪质量指标：

```text
matched_points >= min_matched_points
mean_abs_point_plane <= max_mean_abs_point_plane
min_normal_eigenvalue >= min_normal_eigenvalue
```

以下任一情况立即判定本次初始化失败：

- 状态、位姿、点或残差出现 NaN/Inf；
- 附近不存在历史关键帧；
- 局部参考地图为空；
- 点到面线性系统或 `icp_normal()`求解失败；
- 校正位移或旋转超过主方案中 Session 对齐模块的公共可配置安全上限；本增量设计不重复定义第二套门限；
- 验证超过 `direct_pose_validation_timeout_sec`；
- 验证期间收到更新的 RViz 请求时，旧请求被取消，并立即转为处理最新请求；它不触发 BTC 回退。

软质量必须连续满足指定帧数。中间出现一次软失败时连续计数清零；硬失败立即退出本次验证。

所有失败路径都必须记录可区分的原因。失败不能修改地图包、历史关键帧、历史 BTC 描述子或磁盘文件。

## 10. 只读保证

- 历史二维/三维索引在加载后只读。
- 局部参考地图只由历史关键帧构建。
- 当前 Scan 可以进入独立临时匹配缓存，但禁止插入历史固定层。
- RViz 初始化、BTC 对齐和 LOST 恢复均不得修改 `SavedMap`。
- 定位模式退出时不运行地图导出或历史 GBA。
- 端到端测试前后比较地图目录哈希，结果必须一致。

## 11. 测试设计

### 单元测试

- 允许集合之外的 BTC 帧不能获得投票。
- 范围外高票候选不能遮蔽范围内正确候选。
- 空允许集合明确返回无候选，不退化成全局搜索。
- 无过滤重载与原始全局搜索结果一致。
- 跨 Session 查询不被 `skip_near_num_`错误过滤。
- 半径按30、60、100 m扩展，并正确执行或禁止全局回退。
- 二维索引返回显式 `keyframe_id`，最近高度查询正确。
- RViz 姿态组合使用消息 yaw 和 IMU roll/pitch，z 使用最近历史关键帧。
- 无效坐标系、NaN、IMU未初始化和附近无地图均明确失败。
- 新 RViz 请求使旧请求、旧 BTC 和旧对齐结果失效。
- 连续质量计数、超时和失败回退状态正确。

### 集成测试

- 在重复结构合成地图中比较全局 BTC 与范围 BTC，确认范围过滤降低错误候选干扰。
- RViz 正确初值经过规定帧数后才发布有效定位。
- RViz 错误初值不能发布有效定位，并自动恢复 BTC。
- `IN_MAP_TRACKING`中发送新 RViz 位姿可触发重新初始化。
- LOST 后自动重新执行 YAML 指定的局部或全局 BTC。
- 定位运行前后地图包文件哈希完全一致。

### 目标机验证

Windows 当前环境不能构建 ROS Noetic 工程。正式完成声明必须具有 Ubuntu 20.04 上的：

```bash
catkin_make --pkg voxel_slam
catkin_make run_tests_voxel_slam
catkin_test_results
```

以及 rosbag 或实车验证日志。没有上述证据时，只能表述为完成源码修改和静态核对。

## 12. 非目标

- 不通过 yaw 限制局部 BTC 候选。
- 不建立每次查询专用的临时 `STDescManager`。
- 不先做全局 BTC 再丢弃范围外最佳结果。
- 不把 RViz 输入直接视为定位成功。
- 不增加无 BTC 构建模式。
- 不改变历史地图只读原则、地图包格式核心约束或 GTSAM 对齐方案。
