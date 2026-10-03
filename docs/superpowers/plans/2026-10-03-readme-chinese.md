# README 中文化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将项目根目录 `README.md` 的说明文字完整翻译为中文，同时保持所有可执行命令、配置键、话题名、链接和图片引用准确不变。

**Architecture:** 这是单文件文档改造。先记录原文中的 URL、命令和代码标识，再整体改写中文内容，最后用结构检查、残留英文扫描和 Git 差异检查验证。

**Tech Stack:** Markdown、PowerShell、Git

## Global Constraints

- 技术专有名词、项目名和数据集名保留原文。
- shell 命令、路径、YAML 键、ROS 参数及话题名不得翻译。
- 原有 URL 和图片引用不得丢失。
- 允许修正明显的原文拼写及命令错误，但不得修改程序行为。

---

### Task 1: 将 README 完整翻译为中文

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: 当前 README 的项目介绍、安装命令、数据集运行方法、地图定位流程和评估说明。
- Produces: 中文版 `README.md`，供用户直接完成编译、建图、定位和评估。

- [ ] **Step 1: 记录不可变内容**

运行：

```powershell
rg -o 'https?://[^)> ]+' README.md
rg -n 'roslaunch|rosbag play|rosparam set|catkin_make|python3' README.md
```

预期：列出原有下载链接、论文链接、项目链接以及全部可执行命令，供翻译后对照。

- [ ] **Step 2: 翻译 README**

将全部英文标题和说明改写为中文；保留代码块中的命令、配置键和数据文件名。将错误命令：

```bash
roslaunch roslaunch voxel_slam vxlm_avia.launch
```

修正为：

```bash
roslaunch voxel_slam vxlm_avia.launch
```

- [ ] **Step 3: 检查 Markdown 和关键内容**

运行：

```powershell
rg -n '^#{1,6} ' README.md
rg -n 'roslaunch|rosbag play|rosparam set|catkin_make|python3' README.md
rg -n 'https?://' README.md
```

预期：章节结构完整；所有运行命令、链接及定位说明仍然存在。

- [ ] **Step 4: 检查英文残留和格式**

运行：

```powershell
rg -n '\b(The|This|For|Please|After|Before|Other|Run|Build|Introduction)\b' README.md
git diff --check
```

预期：除专有名词、链接和代码外没有成段英文说明；`git diff --check` 无错误。

- [ ] **Step 5: 提交翻译**

```bash
git add README.md
git commit -m "docs: translate README into Chinese"
```
