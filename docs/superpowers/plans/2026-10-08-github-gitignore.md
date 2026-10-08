# Voxel-SLAM GitHub `.gitignore` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the minimal ignore file with a conservative GitHub-ready policy and remove one already-tracked Office temporary file.

**Architecture:** Organize `.gitignore` by artifact source: agent workspaces, ROS/Catkin/CMake, native compilation, Python, datasets/runtime output, generated map packages, IDE/OS files and secrets. Verify both ignored examples and intentionally retained source/config files with Git itself.

**Tech Stack:** Git ignore patterns, PowerShell, ROS1/Catkin project layout.

## Global Constraints

- Keep source, headers, CMake, ROS packages, YAML, launch, tests, README files and formal images tracked.
- Do not use broad `*.bin`, `*.csv`, `*.yaml` or `*.txt` patterns.
- Anchor general data/output directories at repository root.
- Ignore known v2 map artifacts by exact filename or directory name.
- Remove only `VoxelSLAMPointCloud2/icons/classes/.~PPTX Presentation.pptx` from tracked files.
- Do not rewrite Git history, create a remote or push.

---

### Task 1: Install and verify the GitHub ignore policy

**Files:**
- Modify: `.gitignore`
- Delete: `VoxelSLAMPointCloud2/icons/classes/.~PPTX Presentation.pptx`

**Interfaces:**
- Consumes: Git's repository-relative ignore matcher.
- Produces: a documented ignore policy that protects the repository from generated artifacts without hiding required project files.

- [ ] **Step 1: Record the pre-change failing checks**

Run:

```powershell
git check-ignore build/example.o sample.bag sample.pcd spatial_blocks.bin .vscode/settings.json .env
```

Expected: one or more paths are not reported because the minimal current `.gitignore` does not cover them.

- [ ] **Step 2: Replace `.gitignore` with grouped rules**

Use this exact content:

```gitignore
# Codex and local agent workspaces
.worktrees/
.superpowers/

# ROS / Catkin / CMake build products
build/
devel/
install/
log/
logs/
.catkin_tools/
.catkin_workspace
.ros/
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
CTestTestfile.cmake
compile_commands.json
Testing/
Makefile

# C / C++ compiler and linker outputs
*.o
*.obj
*.lo
*.d
*.a
*.lib
*.so
*.so.*
*.dylib
*.dll
*.exe
*.out
*.pdb
*.ilk
*.gcda
*.gcno
*.gcov
core
core.*

# Python caches, test output and virtual environments
__pycache__/
*.py[cod]
*$py.class
.pytest_cache/
.mypy_cache/
.ruff_cache/
.coverage
.coverage.*
htmlcov/
.tox/
.nox/
.venv/
venv/
env/

# ROS recordings, point clouds and runtime logs
*.bag
*.bag.active
*.db3
*.mcap
*.pcd
*.ply
*.las
*.laz
*.log
*.log.*
*.tum

# Generated Voxel-SLAM map packages
spatial_blocks.idx
spatial_blocks.bin
btc_frames.bin
btc_planes/
keyframes/
keyframes.csv

# Repository-local datasets, maps and evaluation output
/bags/
/data/
/datasets/
/maps/
/map_data/
/map_packages/
/output/
/outputs/
/results/
/runtime/

# IDE and editor state
.vscode/
.idea/
.vs/
*.code-workspace
*.suo
*.user
*.userosscache
*.sln.docstates
*~
*.swp
*.swo
*.tmp
*.temp
*.bak
*.orig
*.rej
.~*
~$*
.#*

# Operating-system metadata
.DS_Store
._*
Thumbs.db
Desktop.ini
$RECYCLE.BIN/

# Local environment and private key files
.env
.env.*
!.env.example
*.pem
*.key
```

- [ ] **Step 3: Remove the tracked Office temporary file**

Run:

```powershell
git rm -- "VoxelSLAMPointCloud2/icons/classes/.~PPTX Presentation.pptx"
```

Expected: only the 165-byte temporary file is staged for deletion; the two formal PNG icons remain tracked.

- [ ] **Step 4: Verify ignored and retained examples**

Run:

```powershell
git check-ignore -v build/example.o devel/setup.bash sample.bag sample.mcap sample.pcd spatial_blocks.bin btc_planes/frame.pcd keyframes/0.pcd .vscode/settings.json .env
git check-ignore -q README.md; if ($LASTEXITCODE -eq 0) { exit 1 }
git check-ignore -q VoxelSLAM/config/mid360.yaml; if ($LASTEXITCODE -eq 0) { exit 1 }
git check-ignore -q VoxelSLAM/launch/vxlm_mid360.launch; if ($LASTEXITCODE -eq 0) { exit 1 }
git check-ignore -q VoxelSLAMPointCloud2/icons/classes/VoxelSLAMPointCloud2.png; if ($LASTEXITCODE -eq 0) { exit 1 }
```

Expected: every generated example reports its matching rule; every retained project file exits nonzero from `git check-ignore`.

- [ ] **Step 5: Inspect repository state and commit**

Run:

```powershell
git diff --check
git status --short
git ls-files | Select-String -Pattern '\.~|~\$'
```

Expected: only `.gitignore`, the temporary-file deletion and this workflow's committed documents are relevant; no tracked Office temporary file remains.

Commit:

```powershell
git add .gitignore
git commit -m "chore: prepare repository ignores for GitHub"
```
