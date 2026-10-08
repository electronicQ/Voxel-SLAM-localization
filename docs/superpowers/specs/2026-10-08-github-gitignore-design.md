# Voxel-SLAM GitHub `.gitignore` 设计

## 目标

为公开上传 GitHub 准备一份完整但保守的 `.gitignore`：保留可复现工程所需的源码、配置、launch、测试、文档和图片，同时阻止构建产物、运行数据、生成地图、缓存、IDE 文件和临时文件进入版本库。

## 保留内容

- `VoxelSLAM/` 与 `VoxelSLAMPointCloud2/` 中的源码、头文件、CMake、ROS package、配置、launch、RViz 配置和测试；
- 根目录及 `docs/` 下的中文说明、设计记录与交接文档；
- `figure/` 和插件正式图标；
- 为复现实验而人工编写的小型文本配置。

## 忽略内容

- Catkin/CMake 的 `build`、`devel`、`install`、日志、测试缓存和编译数据库；
- C/C++ 目标文件、库、可执行文件、调试和覆盖率产物；
- Python 字节码、测试缓存、覆盖率文件和虚拟环境；
- ROS bag、rosbag2 数据库、MCAP、点云、轨迹与运行日志；
- v1/v2 定位地图输出，包括关键帧、BTC 平面、BTC 二进制和空间块文件；
- 常见数据集、地图、评测和输出目录；
- VS Code、Visual Studio、JetBrains、操作系统和编辑器临时文件；
- 本地 `.env` 文件、Codex 工作目录和 Superpowers 运行记录。

## 已跟踪临时文件

`.gitignore` 不会影响已经被 Git 跟踪的文件。删除并忽略误提交的 `VoxelSLAMPointCloud2/icons/classes/.~PPTX Presentation.pptx`，保留正式 PNG 图标。

## 安全边界

- 不使用宽泛的 `*.bin`、`*.csv`、`*.yaml` 或 `*.txt` 规则，避免隐藏源码依赖和人工配置；
- 只按已知地图文件名忽略生成的二进制和索引；
- 顶层数据目录使用根目录锚定规则，避免误伤源码中的同名目录；
- 不改写 Git 历史，不自动创建远程仓库或推送。

## 验证

- 用 `git check-ignore -v` 检查代表性的构建、bag、点云、地图、Python、IDE 和临时文件；
- 确认源码、YAML、launch、README、正式 PNG 图标仍未被忽略；
- 检查 Git 状态只包含预期的 `.gitignore` 更新、临时文件删除和流程文档；
- 运行 `git diff --check`。
