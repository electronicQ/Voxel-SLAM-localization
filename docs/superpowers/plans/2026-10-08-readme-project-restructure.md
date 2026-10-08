# README Project Restructure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将根 README 重写为当前 Voxel-SLAM 二次开发工程的公开技术文档，使读者先完成运行，再理解建图、双通道重定位、空间块连续定位和高频里程计原理。

**Architecture:** 仅修改根目录 `README.md`，以“项目说明—运行流程—技术方案—接口与验证—上游致谢”为主线。保留现有双通道重定位总览和 BTC 完整链路的全部技术信息，但改用正式工程标题并消除重复内容。

**Tech Stack:** Markdown、ROS1 Noetic、Voxel-SLAM、PowerShell/`rg` 文档契约检查、Python 3.12 单元测试。

## Global Constraints

- 本次不修改 C++、Python、YAML、launch 或构建行为。
- README 第一屏必须明确声明本项目基于上游 Voxel-SLAM 二次开发。
- 运行流程必须位于技术原理之前，并按建图、地图导出、定位、有效性检查、导航接入的顺序编写。
- 必须完整保留“双通道重定位总览”和 BTC 自动重定位完整链路的流程、坐标系、公式和 GTSAM 优化边界。
- README 不得出现“面试”“速记”“方便复习”等个人语境措辞。
- 原版仓库、论文、视频和竞赛信息移动到文末“上游项目与致谢”。
- 不添加未经 Ubuntu 实机验证的性能或成功率结论。

---

### Task 1: 重写并验证根 README

**Files:**
- Modify: `README.md`
- Reference: `docs/superpowers/specs/2026-10-08-readme-project-restructure-design.md`
- Reference: `VoxelSLAM/config/mid360.yaml`
- Reference: `VoxelSLAM/config/localization_mid360.yaml`
- Reference: `VoxelSLAM/config/scan_planner_adapter.yaml`
- Reference: `VoxelSLAM/launch/vxlm_mid360.launch`
- Reference: `VoxelSLAM/launch/vxlm_localization_mid360.launch`
- Reference: `VoxelSLAM/launch/scan_planner_adapter.launch`
- Reference: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: 当前配置中的模式、地图路径、BTC、RViz、空间块和里程计参数；当前代码发布的话题与 TF frame。
- Produces: 一份可独立指导建图、定位和导航接入的中文根 README。

- [ ] **Step 1: 运行重构前文档契约检查并确认当前结构不满足要求**

Run:

```powershell
rg -n "^# Voxel-SLAM：建图、重定位与连续定位系统" README.md
rg -n "面试速记" README.md
```

Expected: 第一条无匹配；第二条匹配现有“面试速记：BTC 重定位完整链路”，证明新标题和正式措辞尚未落实。

- [ ] **Step 2: 用项目化结构重写 `README.md`**

README 使用以下一级、二级结构，标题文字保持一致：

```markdown
# Voxel-SLAM：建图、重定位与连续定位系统

## 1. 项目简介
## 2. 主要功能
## 3. 环境与编译
## 4. 完整运行流程
### 4.1 建图配置
### 4.2 运行建图并导出 v2 地图
### 4.3 转换已有 v1 地图（可选）
### 4.4 启动定位
### 4.5 完成初始重定位
### 4.6 检查定位状态
### 4.7 接入 SCAN-Planner（可选）
## 5. 建图与地图准备原理
## 6. 重定位技术方案
### 6.1 双通道重定位总览
### 6.2 BTC 自动重定位
### 6.3 BTC 自动重定位完整链路
### 6.4 RViz 人工初始化
### 6.5 两条通道的公共汇合路径
## 7. 连续定位技术方案
## 8. IMU 高频里程计与导航接口
## 9. 参数、话题与 TF
## 10. 验证、评估与常见问题
## 11. 上游项目与致谢
```

具体内容要求：

- “项目简介”明确原版能力与当前改造边界，突出 v2 空间块地图、BTC/RViz 双通道、连续定位和高频里程计。
- “完整运行流程”保留可复制的 `roslaunch`、`rosbag play`、`rosparam set finish true`、`build_spatial_map`、状态检查和 SCAN-Planner 命令。
- “建图与地图准备原理”说明最终优化后生成静态空间块，并解释 `manifest.yaml`、`keyframes.csv`、`keyframes/`、`btc_frames.bin`、`btc_planes/`、`spatial_blocks.idx`、`spatial_blocks.bin`。
- “双通道重定位总览”保留现有从 `LOCAL_ODOMETRY` 分叉、再汇合到 `PREPARING_LOCAL_MAP` 和 `IN_MAP_TRACKING` 的文本图。
- “BTC 自动重定位完整链路”保留默认 10 帧 `ScanPose`、`GenerateSTDescs()`、BTC 候选、平面 ICP、GTSAM 当前 Session 对齐、空间块加载的文本流程；保留 `W/O/H/C` 与以下公式：

```text
T_W_C = T_W_H × T_H_C
T_W_O = T_W_C × inverse(T_O_C)
```

- “RViz 人工初始化”说明 x/y/yaw、IMU roll/pitch、最近历史关键帧 z 和连续局部地图验证。
- “连续定位技术方案”说明规则块坐标枚举、哈希索引、后台加载、不可变快照、加载/卸载滞回、地图缺口回滚和 `LOST` 恢复。
- “IMU 高频里程计”区分权威 IESKF 与只传播均值的独立预测器，说明后验刷新、IMU 尾队列重放、`max_prediction_sec` 和输出门控。
- “参数、话题与 TF”集中列出 `/localization/state`、`/localization/valid`、`/map_scan`、`/map_path`、`/LIO/odom_imu`、`/LIO/odom_vehicle` 与 `camera_init -> aft_mapped -> base_link`。
- “上游项目与致谢”保留上游 GitHub、arXiv、YouTube、HILTI 2023 和 ICCV 2023 信息；删除上游数据集逐包运行教程。

- [ ] **Step 3: 运行新结构和禁止措辞检查**

Run:

```powershell
rg -n "^# Voxel-SLAM：建图、重定位与连续定位系统|^## [1-9]|^## 10\.|^## 11\." README.md
rg -n "面试|速记|方便复习" README.md
```

Expected: 第一条列出标题和 11 个一级章节；第二条无匹配。

- [ ] **Step 4: 验证关键技术链路和接口仍完整**

Run:

```powershell
$required = @(
  '双通道重定位总览', 'BTC 自动重定位完整链路',
  'LOCAL_ODOMETRY', 'PREPARING_LOCAL_MAP', 'IN_MAP_TRACKING',
  'T_W_C = T_W_H', 'T_W_O = T_W_C',
  'GenerateSTDescs', 'icp_normal', 'GTSAM',
  '/localization/state', '/localization/valid', '/map_scan', '/map_path',
  '/LIO/odom_imu', '/LIO/odom_vehicle',
  'camera_init -> aft_mapped -> base_link'
)
foreach ($item in $required) {
  if (-not (Select-String -LiteralPath README.md -SimpleMatch $item -Quiet)) {
    throw "README 缺少：$item"
  }
}
```

Expected: 命令成功退出，无缺失项。

- [ ] **Step 5: 验证 README 引用的仓库文件与代码契约**

Run:

```powershell
$files = @(
  'VoxelSLAM/config/mid360.yaml',
  'VoxelSLAM/config/localization_mid360.yaml',
  'VoxelSLAM/config/scan_planner_adapter.yaml',
  'VoxelSLAM/launch/vxlm_mid360.launch',
  'VoxelSLAM/launch/vxlm_localization_mid360.launch',
  'VoxelSLAM/launch/scan_planner_adapter.launch',
  'VoxelSLAM/scripts/evaluate_localization.py'
)
foreach ($file in $files) {
  if (-not (Test-Path -LiteralPath $file -PathType Leaf)) {
    throw "README 引用了不存在的文件：$file"
  }
}
rg -n "use_prior_pose_for_btc|rviz_initialpose_enabled|direct_pose_validation_scans|load_radius|unload_radius|max_prediction_sec" VoxelSLAM/config
rg -n '"/localization/state"|"/localization/valid"|"/map_scan"|"/map_path"|"/LIO/odom_imu"|"/LIO/odom_vehicle"' VoxelSLAM/src
```

Expected: 全部文件存在，参数和话题在配置或源码中有匹配。

- [ ] **Step 6: 检查 Markdown 完整性和项目测试**

Run:

```powershell
$text = Get-Content -Raw -Encoding UTF8 README.md
$fences = ([regex]::Matches($text, '(?m)^```')).Count
if ($fences % 2 -ne 0) { throw "README 代码围栏数量不是偶数：$fences" }
git diff --check
py -3.12 -m unittest discover -s VoxelSLAM/test -p "*_test.py"
```

Expected: 代码围栏成对，`git diff --check` 成功，20 项 Python 合同/评估测试通过。

- [ ] **Step 7: 提交 README 重构**

```powershell
git add README.md
git commit -m "docs: restructure README around mapping and localization"
```
