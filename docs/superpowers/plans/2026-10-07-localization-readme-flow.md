# Voxel-SLAM Localization README Flow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the root Chinese README with an implementation-accurate explanation of BTC/RViz initialization, convergence into continuous in-map tracking, and IMU-rate odometry output.

**Architecture:** Keep the root `README.md` as the single detailed source. Expand the existing localization and high-rate odometry subsections in runtime order, while preserving the shorter package README as an entry point.

**Tech Stack:** Markdown, ROS1 terminology, PowerShell/Python static checks, Git.

## Global Constraints

- Describe the current code, not the former BnB/GN/SE-Sync pipeline.
- State that BTC is followed by plane ICP verification and GTSAM Levenberg–Marquardt session alignment.
- State that RViz bypasses BTC but requires consecutive local-map scan validation.
- Keep the authoritative scan-batched IESKF separate from the non-authoritative IMU-rate predictor.
- Do not claim ROS/C++ runtime verification on Windows.

---

### Task 1: Document the complete localization and navigation-output flow

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: the existing sections `4.5.3` through `4.5.8` and the implemented ROS topics/state names.
- Produces: one Chinese operational narrative covering initialization, tracking, recovery and high-rate output.

- [ ] **Step 1: Run the documentation contract before editing**

Run:

```powershell
python -c "from pathlib import Path; s=Path('README.md').read_text(encoding='utf-8'); required=['BTC 自动重定位通道','GTSAM Levenberg–Marquardt','RViz 人工初始化通道','两条通道的公共汇合路径','连续 in-map tracking','正式 IESKF 路径','IMU 高频预测路径']; missing=[x for x in required if x not in s]; print(missing); raise SystemExit(bool(missing))"
```

Expected: non-zero exit with one or more missing headings or phrases.

- [ ] **Step 2: Expand the localization flow**

Under the existing localization section, add these exact subsections in order:

```markdown
##### 双通道重定位总览
##### BTC 自动重定位通道
##### RViz 人工初始化通道
##### 两条通道的公共汇合路径
##### 连续 in-map tracking
##### 失效检测与自动恢复
```

Describe the state flow, transform meaning `T_W_O`, BTC candidate filtering and radius expansion, BTC/ICP/GTSAM alignment, RViz six-degree-of-freedom seed construction, spatial-block preparation, immutable snapshots, point-to-plane IESKF quality gates and `LOST` recovery.

- [ ] **Step 3: Expand the high-rate odometry flow**

Under `4.5.8`, add these exact subsections:

```markdown
##### 正式 IESKF 路径
##### IMU 高频预测路径
##### 后验快照刷新与输出门控
##### 车辆里程计适配
```

Explain that each IMU message enters both the formal buffer and the independent predictor, the formal path remains authoritative, the fast path uses sensor timestamps, replayed posterior snapshots and a 0.5-second horizon, and the adapter composes the static `aft_mapped -> base_link` transform without broadcasting another dynamic TF.

- [ ] **Step 4: Run the documentation contract and regression checks**

Run:

```powershell
python -c "from pathlib import Path; s=Path('README.md').read_text(encoding='utf-8'); required=['BTC 自动重定位通道','GTSAM Levenberg–Marquardt','RViz 人工初始化通道','两条通道的公共汇合路径','连续 in-map tracking','正式 IESKF 路径','IMU 高频预测路径']; missing=[x for x in required if x not in s]; print('README FLOW PASS' if not missing else missing); raise SystemExit(bool(missing))"
python -m unittest discover -s VoxelSLAM/test -p "*_test.py"
git diff --check
```

Expected: documentation contract passes, all current Python tests pass, and `git diff --check` reports no errors.

- [ ] **Step 5: Commit**

```powershell
git add README.md
git commit -m "docs: explain localization and odometry data flow"
```
