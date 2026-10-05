# 三维空间体素块地图 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将定位模式的历史关键帧局部地图完整重建，替换为离线生成、按空间异步加载和卸载的三维只读体素块地图。

**Architecture:** 地图包 v2 保留关键帧/BTC 重定位层，并新增 `20 m × 20 m × 10 m` 空间块层。块内保存预构建的 `1 m` 根体素、静态八叉树和不确定性平面；定位运行时通过规则块索引和哈希目录按需加载，IESKF 每个 Scan 持有一份不可变活动地图快照。

**Tech Stack:** C++14、ROS Noetic、PCL、Eigen、Boost.Filesystem、GTest、标准库线程与 `shared_ptr`。

## Global Constraints

- 所有实现位于 `codex/spatial-voxel-blocks` 分支，不创建额外 worktree。
- 地图块默认尺寸必须是 `20.0 m × 20.0 m × 10.0 m`，融合分辨率 `0.1 m`，根体素尺寸 `1.0 m`。
- 块尺寸必须是根体素尺寸的整数倍；导出和查询必须共用同一套负坐标安全索引函数。
- 连续定位不得将当前扫描写入历史地图，不得因块集合变化重建全局 `surf_map`。
- 建图模式原有滑窗 `surf_map`、LBA、BTC、PGO 和 GBA 行为保持不变。
- 地图包 v2 继续保存关键帧、BTC 描述子和 BTC 验证平面；空间块仅替换连续定位参考地图。
- 空间块二进制逐字段序列化，不保存裸指针、容器布局或编译器填充。
- 定位进程拒绝 v1 地图；独立工具负责 v1 到 v2 转换，输入目录不能被覆盖。
- 每个 Scan 使用固定只读快照，后台发布不得使 Scan 内缓存指针失效。
- `preload/load/unload` 默认半径分别为 `15/30/40 m`，运动前视 `2 s`，地图缺口超时 `1 s`，初始预加载超时 `10 s`。
- Windows 没有 ROS Noetic/C++ 编译环境，只允许声明静态检查和 Python 测试结果；全部 C++ GTest、编译和 rosbag 实时性必须在 Ubuntu 20.04 + ROS Noetic 复验。

---

## File Structure

- `VoxelSLAM/src/spatial_map_types.hpp/.cpp`：空间索引、配置、只读节点/平面/块数据结构和共享坐标转换。
- `VoxelSLAM/src/spatial_map_builder.hpp/.cpp`：跨关键帧融合和静态八叉树构建。
- `VoxelSLAM/src/spatial_map_matcher.hpp/.cpp`：静态块两级索引和点到面查询。
- `VoxelSLAM/src/spatial_map_io.hpp/.cpp`：`spatial_blocks.idx/.bin` 的确定性序列化、校验和随机块读取。
- `VoxelSLAM/src/spatial_map_manager.hpp/.cpp`：后台加载队列、加载/卸载滞回、Session 代数和活动快照。
- `VoxelSLAM/src/build_spatial_map.cpp`：v1 到 v2 离线转换命令。
- `VoxelSLAM/src/map_package.hpp/.cpp`：地图包 v2 清单、空间块导出和 v2 加载入口；保留仅供转换器使用的 v1 读取入口。
- `VoxelSLAM/src/localization_state.hpp/.cpp`：增加 `PREPARING_LOCAL_MAP` 及地图准备事件。
- `VoxelSLAM/src/voxelslam.cpp`：参数接线、异步地图状态轮询、静态匹配器接入 IESKF。
- `VoxelSLAM/test/spatial_map_types_test.cpp`：索引、边界和加载球测试。
- `VoxelSLAM/test/spatial_map_builder_test.cpp`：跨帧融合、静态树和平面匹配测试。
- `VoxelSLAM/test/spatial_map_matcher_test.cpp`：静态点面查询、缓存和原版门限等价测试。
- `VoxelSLAM/test/spatial_map_io_test.cpp`：二进制往返和损坏输入测试。
- `VoxelSLAM/test/spatial_map_manager_test.cpp`：加载、卸载、快照生命周期和代数测试。
- `VoxelSLAM/test/map_package_roundtrip_test.cpp`：地图包 v2 和 v1 拒绝/转换测试。
- `VoxelSLAM/test/localization_state_test.cpp`：地图准备和缺口超时状态测试。
- `VoxelSLAM/config/localization_mid360.yaml`、`VoxelSLAM/config/mid360.yaml`：地图格式参数与定位加载参数。
- `README.md`：中文地图生成、转换、定位运行和 Ubuntu 验证说明。

---

### Task 1: 规则三维块索引与配置校验

**Files:**
- Create: `VoxelSLAM/src/spatial_map_types.hpp`
- Create: `VoxelSLAM/src/spatial_map_types.cpp`
- Create: `VoxelSLAM/test/spatial_map_types_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Produces: `BlockIndex`、`RootVoxelIndex`、`BlockAabb`、`SpatialMapConfig`、`floor_grid_index()`、`world_to_block()`、`world_to_root_voxel()`、`root_voxel_to_block()`、`blocks_intersecting_sphere()`。
- `BlockIndex` 和 `RootVoxelIndex` 使用有符号 64 位整数，并提供相等、严格排序和哈希。

- [ ] **Step 1: 写索引边界失败测试**

```cpp
TEST(SpatialMapGeometry, FloorsNegativeCoordinatesAndKeepsExactBoundaries) {
  EXPECT_EQ(floor_grid_index(0.0, 20.0), 0);
  EXPECT_EQ(floor_grid_index(19.999, 20.0), 0);
  EXPECT_EQ(floor_grid_index(20.0, 20.0), 1);
  EXPECT_EQ(floor_grid_index(-0.001, 20.0), -1);
  EXPECT_EQ(floor_grid_index(-20.0, 20.0), -1);
  EXPECT_EQ(floor_grid_index(-20.001, 20.0), -2);
}

TEST(SpatialMapGeometry, RejectsMisalignedOrNonFiniteConfiguration) {
  SpatialMapConfig config;
  std::string error;
  EXPECT_TRUE(config.validate(error)) << error;
  config.size_z = 10.5;
  EXPECT_FALSE(config.validate(error));
  EXPECT_NE(error.find("integer multiple"), std::string::npos);
}
```

- [ ] **Step 2: 在 Ubuntu 运行测试并确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_types_test`

Expected: FAIL，提示 `spatial_map_types.hpp` 或接口不存在。Windows 记录 `NOT RUN: ROS Noetic toolchain unavailable`，并用 `rg -n "floor_grid_index|blocks_intersecting_sphere" VoxelSLAM/test/spatial_map_types_test.cpp` 确认测试已落盘。

- [ ] **Step 3: 实现最小索引和球—AABB 枚举**

```cpp
struct SpatialMapConfig {
  double size_x = 20.0;
  double size_y = 20.0;
  double size_z = 10.0;
  double fusion_leaf_size = 0.1;
  double root_voxel_size = 1.0;
  int max_layer = 2;
  double min_eigen_value = 0.0025;
  std::array<int, 4> min_points{{5, 5, 5, 5}};
  std::array<double, 4> plane_ratio_threshold{{0.25, 0.25, 0.25, 0.25}};
  bool save_display_cloud = false;
  bool validate(std::string &error) const;
};

struct BlockIndex {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;
  BlockIndex() = default;
  BlockIndex(std::int64_t x_value, std::int64_t y_value,
             std::int64_t z_value)
      : x(x_value), y(y_value), z(z_value) {}
};

std::int64_t floor_grid_index(double coordinate, double resolution);
BlockIndex world_to_block(const Eigen::Vector3d &point,
                          const SpatialMapConfig &config);
std::vector<BlockIndex> blocks_intersecting_sphere(
    const Eigen::Vector3d &center, double radius,
    const SpatialMapConfig &config);
```

实现先枚举球外接长方体内的整数块坐标，再使用点到 AABB 的最小平方距离过滤；输出按 `x/y/z` 字典序排序，保证确定性。

- [ ] **Step 4: 增加精确相交和哈希去重测试并运行 GREEN**

```cpp
TEST(SpatialMapGeometry, EnumeratesOnlyBlocksWhoseAabbTouchesSphere) {
  SpatialMapConfig config;
  const auto blocks = blocks_intersecting_sphere(
      Eigen::Vector3d(45.0, 12.0, 3.0), 30.0, config);
  EXPECT_NE(std::find(blocks.begin(), blocks.end(), BlockIndex{2, 0, 0}),
            blocks.end());
  for (const BlockIndex &index : blocks) {
    EXPECT_TRUE(sphere_intersects_aabb(Eigen::Vector3d(45.0, 12.0, 3.0),
                                       30.0, block_aabb(index, config)));
  }
}
```

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_types_test`

Expected: PASS。Windows 执行 `git diff --check` 与接口/测试静态检查，不宣称 C++ 通过。

- [ ] **Step 5: 提交**

```bash
git add VoxelSLAM/src/spatial_map_types.hpp VoxelSLAM/src/spatial_map_types.cpp VoxelSLAM/test/spatial_map_types_test.cpp VoxelSLAM/CMakeLists.txt
git commit -m "feat: add spatial map grid geometry"
```

### Task 2: 跨关键帧融合、静态八叉树与点面查询

**Files:**
- Create: `VoxelSLAM/src/spatial_map_builder.hpp`
- Create: `VoxelSLAM/src/spatial_map_builder.cpp`
- Create: `VoxelSLAM/src/spatial_map_matcher.hpp`
- Create: `VoxelSLAM/src/spatial_map_matcher.cpp`
- Create: `VoxelSLAM/test/spatial_map_builder_test.cpp`
- Create: `VoxelSLAM/test/spatial_map_matcher_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 的索引和 `SpatialMapConfig`。
- Produces: `SpatialMapPoint`、`StaticPlane`、`StaticOctreeNode`、`RootVoxelRecord`、`StaticMapBlock`、`SpatialMapBuilder::build()`、`StaticMapMatcher::match()`。

- [ ] **Step 1: 写融合和静态匹配失败测试**

```cpp
TEST(SpatialMapBuilder, FusesRepeatedObservationsAcrossSources) {
  SpatialMapConfig config;
  std::vector<SpatialMapPoint> points;
  points.push_back(testPoint(1.01, 2.01, 0.0));
  points.push_back(testPoint(1.04, 2.03, 0.0));
  points.push_back(testPoint(1.08, 2.07, 0.0));
  SpatialMapBuildResult result;
  std::string error;
  ASSERT_TRUE(SpatialMapBuilder::build(points, config, result, error)) << error;
  EXPECT_EQ(result.fused_point_count, 1U);
}

TEST(SpatialMapBuilder, MatchesAPlaneThroughTheOwningBlockAndOctree) {
  const SpatialMapBuildResult result = buildHorizontalPlaneFixture();
  StaticPlaneMatch match;
  StaticMapMatcher matcher(result.config);
  ASSERT_TRUE(matcher.match(result.blocks.at(0),
                            Eigen::Vector3d(0.2, 0.2, 0.01),
                            Eigen::Matrix3d::Identity() * 1e-4, nullptr,
                            match));
  EXPECT_NEAR(std::abs(match.normal.z()), 1.0, 1e-6);
  EXPECT_TRUE(std::isfinite(match.sigma_d));
}
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_builder_test run_tests_voxel_slam_gtest_spatial_map_matcher_test`

Expected: FAIL，类型和构建器尚不存在。

- [ ] **Step 3: 实现确定性融合和静态树**

```cpp
struct SpatialMapPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  float intensity = 0.0F;
};

struct StaticOctreeNode {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double half_length = 0.0;
  std::array<std::uint32_t, 8> children;
  std::uint32_t plane_index = kInvalidSpatialIndex;
};

struct StaticPlane {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
  double radius = 0.0;
};

struct SpatialMapBuildResult {
  SpatialMapConfig config;
  std::size_t source_point_count = 0;
  std::size_t fused_point_count = 0;
  std::vector<StaticMapBlock> blocks;
};
```

融合键使用 `floor_grid_index(position, fusion_leaf_size)`；代表点取位置、协方差和强度均值。每个八叉树节点累计 `N/Σp/Σppᵀ` 和与原版 `Bf_var()` 等价的协方差项，使用相同的特征值和平面判定公式。只序列化包含有效平面或有效子节点的节点。

- [ ] **Step 4: 增加确定性、负坐标和损坏数据测试并运行 GREEN**

测试同一输入乱序两次生成的块、根体素、节点和平面逐字段一致；非有限点、零点、配置越界必须原子失败，不修改调用方结果。

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_builder_test run_tests_voxel_slam_gtest_spatial_map_matcher_test`

Expected: PASS。

- [ ] **Step 5: 提交**

```bash
git add VoxelSLAM/src/spatial_map_builder.hpp VoxelSLAM/src/spatial_map_builder.cpp VoxelSLAM/src/spatial_map_matcher.hpp VoxelSLAM/src/spatial_map_matcher.cpp VoxelSLAM/test/spatial_map_builder_test.cpp VoxelSLAM/test/spatial_map_matcher_test.cpp VoxelSLAM/CMakeLists.txt
git commit -m "feat: build immutable spatial voxel blocks"
```

### Task 3: 空间块索引与二进制随机读取

**Files:**
- Create: `VoxelSLAM/src/spatial_map_io.hpp`
- Create: `VoxelSLAM/src/spatial_map_io.cpp`
- Create: `VoxelSLAM/test/spatial_map_io_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `StaticMapBlock` 和 `SpatialMapConfig`。
- Produces: `SpatialBlockIndexEntry`、`SpatialMapCatalog`、`SpatialMapIO::save()`、`load_catalog()`、`load_block()`、`validate_all_blocks()`。

- [ ] **Step 1: 写文件往返和损坏输入失败测试**

```cpp
TEST(SpatialMapIO, RoundTripsCatalogAndRandomBlock) {
  const std::vector<StaticMapBlock> blocks = twoBlockFixture();
  TemporaryDirectory temp;
  SpatialMapCatalog written;
  std::string error;
  ASSERT_TRUE(SpatialMapIO::save(temp.path(), blocks, testConfig(), written,
                                 error)) << error;
  SpatialMapCatalog loaded;
  ASSERT_TRUE(SpatialMapIO::load_catalog(temp.path(), loaded, error)) << error;
  StaticMapBlock block;
  ASSERT_TRUE(SpatialMapIO::load_block(temp.path(), loaded,
                                       BlockIndex{1, -2, 0}, block, error))
      << error;
  EXPECT_EQ(block.index, BlockIndex(1, -2, 0));
}
```

另写截断 `.idx`、越界 offset/length、校验码错误、非法节点下标、重复根体素测试，均要求失败且不修改输出对象。

- [ ] **Step 2: 运行测试确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_io_test`

Expected: FAIL，`spatial_map_io.hpp` 不存在。

- [ ] **Step 3: 实现逐字段小端格式和 CRC32**

```cpp
struct SpatialBlockIndexEntry {
  BlockIndex index;
  BlockAabb bounds;
  std::uint64_t offset = 0;
  std::uint64_t length = 0;
  std::uint32_t checksum = 0;
  std::uint32_t root_count = 0;
  std::uint32_t node_count = 0;
  std::uint32_t plane_count = 0;
  std::uint32_t display_point_count = 0;
};

struct SpatialMapCatalog {
  SpatialMapConfig config;
  std::vector<SpatialBlockIndexEntry> entries;
  std::unordered_map<BlockIndex, std::size_t, BlockIndexHash> lookup;
};
```

文件魔数固定为 `VXBLKIDX` 和 `VXBLKDAT`，版本固定为 `1`。写入临时文件，关闭并全量验证后再改名。读取前检查文件大小、计数乘法溢出、offset 加 length 溢出和配置上限；运行时根体素哈希只在块校验通过后建立。

- [ ] **Step 4: 运行完整 I/O 测试 GREEN**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_io_test`

Expected: PASS。

- [ ] **Step 5: 提交**

```bash
git add VoxelSLAM/src/spatial_map_io.hpp VoxelSLAM/src/spatial_map_io.cpp VoxelSLAM/test/spatial_map_io_test.cpp VoxelSLAM/CMakeLists.txt
git commit -m "feat: serialize spatial voxel block maps"
```

### Task 4: 地图包 v2、直接导出和 v1 离线转换

**Files:**
- Modify: `VoxelSLAM/src/map_package.hpp`
- Modify: `VoxelSLAM/src/map_package.cpp`
- Create: `VoxelSLAM/src/build_spatial_map.cpp`
- Modify: `VoxelSLAM/test/map_package_roundtrip_test.cpp`
- Create: `VoxelSLAM/test/spatial_map_conversion_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 2 构建器和 Task 3 I/O。
- Produces: `LoadedMapPackage`，v2 `MapPackageIO::save/load()`，仅转换器可调用的 `MapPackageIO::load_legacy_v1()`，`build_spatial_map` 可执行文件。

- [ ] **Step 1: 将地图包测试改为期望 v2 并增加旧版本拒绝测试**

```cpp
TEST(MapPackageIO, SavesVersionTwoWithSpatialArtifacts) {
  TemporaryMapPath temp;
  MapPackageOptions options = makeOptions();
  options.spatial_map = testSpatialConfig();
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(temp.map_path(), makePlanarMap(), options,
                                 error)) << error;
  EXPECT_TRUE(fs::is_regular_file(temp.path() / "spatial_blocks.idx"));
  EXPECT_TRUE(fs::is_regular_file(temp.path() / "spatial_blocks.bin"));
  EXPECT_NE(readFile(temp.path() / "manifest.yaml").find("version: 2"),
            std::string::npos);
}

TEST(MapPackageIO, LocalizationLoadRejectsLegacyVersionWithConverterHint) {
  TemporaryMapPath temp;
  writeLegacyV1Fixture(temp.path());
  LoadedMapPackage loaded;
  std::string error;
  EXPECT_FALSE(MapPackageIO::load(temp.map_path(), loaded, makeOptions(), error));
  EXPECT_NE(error.find("build_spatial_map"), std::string::npos);
}
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_map_package_roundtrip_test run_tests_voxel_slam_gtest_spatial_map_conversion_test`

Expected: FAIL，当前清单仍为 v1 且没有空间块文件。

- [ ] **Step 3: 扩展地图包接口并实现 v2 原子导出**

```cpp
struct LoadedMapPackage {
  SavedMap historical_map;
  SpatialMapCatalog spatial_catalog;
  std::string directory;
};

struct MapPackageOptions {
  int win_size = 10;
  double keyframe_distance_m = 0.1;
  double keyframe_angle_deg = 5.0;
  double keyframe_voxel_size = 0.1;
  ConfigSetting btc_config{};
  SpatialMapConfig spatial_map{};
};
```

`save()` 在同一临时包目录中：保存关键帧/BTC，转换所有关键帧点及协方差到 `map`，构建空间块，写 `.idx/.bin`，最后写 v2 清单并原子发布。`load()` 先验证 v2 清单和指纹，再加载关键帧/BTC及轻量空间目录，不读取所有块。空间几何和静态八叉树参数以 v2 清单为唯一权威值；定位 YAML 不能覆盖它们，BTC 指纹兼容检查继续保留。

- [ ] **Step 4: 实现安全 v1 转换器**

转换器解析 `--input`、`--output` 及可选空间块参数；拒绝相同或嵌套输出目录。`load_legacy_v1()` 仅供该工具使用，保留 v1 清单中的 BTC 指纹和关键帧参数，不要求定位运行时配置恰好等于旧地图；转换完成后调用 `validate_all_blocks()`，失败时删除临时输出而不修改输入。CMake 为转换器增加可执行目标和安装规则。

- [ ] **Step 5: 运行地图包和转换测试 GREEN**

Run: `catkin_make run_tests_voxel_slam_gtest_map_package_roundtrip_test run_tests_voxel_slam_gtest_spatial_map_conversion_test`

Expected: PASS。

- [ ] **Step 6: 提交**

```bash
git add VoxelSLAM/src/map_package.hpp VoxelSLAM/src/map_package.cpp VoxelSLAM/src/build_spatial_map.cpp VoxelSLAM/test/map_package_roundtrip_test.cpp VoxelSLAM/test/spatial_map_conversion_test.cpp VoxelSLAM/CMakeLists.txt
git commit -m "feat: upgrade map packages to spatial map v2"
```

### Task 5: 后台块管理器与每 Scan 活动快照

**Files:**
- Create: `VoxelSLAM/src/spatial_map_manager.hpp`
- Create: `VoxelSLAM/src/spatial_map_manager.cpp`
- Create: `VoxelSLAM/test/spatial_map_manager_test.cpp`
- Modify: `VoxelSLAM/CMakeLists.txt`

**Interfaces:**
- Consumes: `SpatialMapCatalog`、`SpatialMapIO::load_block()` 和 Task 1 的范围枚举。
- Produces: `SpatialMapRuntimeConfig`、`SpatialMapSnapshot`、`SpatialMapStatus`、`SpatialMapManager`。

- [ ] **Step 1: 写加载、滞回、快照和过期代数失败测试**

```cpp
TEST(SpatialMapManager, LoadsCoreThenPrefetchesAndUnloadsOutsideHysteresis) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const std::uint64_t generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), generation);
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(testTimeout()));
  EXPECT_TRUE(manager.status().core_ready);
  const auto first = manager.snapshot();
  EXPECT_FALSE(first->blocks.empty());

  manager.request(Eigen::Vector3d(100.0, 0.0, 0.0),
                  Eigen::Vector3d::Zero(), generation);
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(testTimeout()));
  EXPECT_NE(first->blocks.find(BlockIndex{0, 0, 0}), first->blocks.end());
  const auto second = manager.snapshot();
  EXPECT_NE(second.get(), first.get());
  EXPECT_EQ(second->blocks.find(BlockIndex{0, 0, 0}), second->blocks.end());
}

TEST(SpatialMapManager, DiscardsLoadsFromAnOldSessionGeneration) {
  auto source = std::make_shared<ControllableBlockSource>(catalogFixture());
  SpatialMapManager manager(source, runtimeConfig());
  const auto old_generation = manager.begin_session();
  manager.request(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  old_generation);
  const auto new_generation = manager.begin_session();
  source->release_all();
  ASSERT_TRUE(manager.wait_until_idle(testTimeout()));
  EXPECT_EQ(manager.status().generation, new_generation);
  EXPECT_TRUE(manager.snapshot()->blocks.empty());
}
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_manager_test`

Expected: FAIL，管理器不存在。

- [ ] **Step 3: 实现可注入块源和后台工作线程**

```cpp
struct SpatialMapRuntimeConfig {
  double preload_radius = 15.0;
  double load_radius = 30.0;
  double unload_radius = 40.0;
  double prefetch_time_sec = 2.0;
  double map_gap_timeout_sec = 1.0;
  double preload_timeout_sec = 10.0;
  int max_background_loads = 2;
  bool validate(std::string &error) const;
};

struct SpatialMapSnapshot {
  std::uint64_t generation = 0;
  std::unordered_map<BlockIndex, std::shared_ptr<const StaticMapBlock>,
                     BlockIndexHash> blocks;
};
```

工作线程只修改私有 staging 集合；发布时构造新的轻量 `SpatialMapSnapshot`。`snapshot()` 在短锁内复制一个 `shared_ptr` 后立即返回，点级查询不得持锁。请求合并时核心块优先、AABB 距离次序；析构必须停止并 `join()` 线程。

- [ ] **Step 4: 增加损坏块、加载错误、析构和并发快照测试并运行 GREEN**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_manager_test`

Expected: PASS，ThreadSanitizer 环境额外运行同一测试。

- [ ] **Step 5: 提交**

```bash
git add VoxelSLAM/src/spatial_map_manager.hpp VoxelSLAM/src/spatial_map_manager.cpp VoxelSLAM/test/spatial_map_manager_test.cpp VoxelSLAM/CMakeLists.txt
git commit -m "feat: stream immutable spatial map snapshots"
```

### Task 6: 地图准备状态和加载缺口事件

**Files:**
- Modify: `VoxelSLAM/src/localization_state.hpp`
- Modify: `VoxelSLAM/src/localization_state.cpp`
- Modify: `VoxelSLAM/test/localization_state_test.cpp`

**Interfaces:**
- Produces: `LocalizationState::PREPARING_LOCAL_MAP`、`LocalizationEvent::LOCAL_MAP_READY`、`LOCAL_MAP_FAILED`、`MAP_GAP_TIMEOUT`。

- [ ] **Step 1: 写 BTC 与 RViz 地图准备失败测试**

```cpp
TEST(LocalizationStateMachine, WaitsForSpatialMapBeforeBecomingValid) {
  LocalizationStateMachine machine(5);
  std::string warning;
  ASSERT_TRUE(machine.handle(LocalizationEvent::MAP_LOADED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_STARTED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::BTC_MATCHED, warning).accepted);
  ASSERT_TRUE(machine.handle(LocalizationEvent::ALIGNMENT_APPLIED, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::PREPARING_LOCAL_MAP);
  EXPECT_FALSE(machine.localization_valid());
  ASSERT_TRUE(machine.handle(LocalizationEvent::LOCAL_MAP_READY, warning).accepted);
  EXPECT_EQ(machine.state(), LocalizationState::IN_MAP_TRACKING);
  EXPECT_TRUE(machine.localization_valid());
}

TEST(LocalizationStateMachine, MapGapTimeoutEntersLostImmediately) {
  LocalizationStateMachine machine(5);
  std::string warning;
  enterTrackingThroughBtc(machine);
  const LocalizationTransition transition =
      machine.handle(LocalizationEvent::MAP_GAP_TIMEOUT, warning);
  EXPECT_EQ(machine.state(), LocalizationState::LOST);
  EXPECT_TRUE(transition.cleanup_requested);
}
```

- [ ] **Step 2: 运行测试确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_localization_state_test`

Expected: FAIL，新状态和事件不存在。

- [ ] **Step 3: 实现状态转换**

BTC 的 `ALIGNMENT_APPLIED` 转入 `PREPARING_LOCAL_MAP`；`LOCAL_MAP_READY` 转入 `IN_MAP_TRACKING`。RViz 的 `ALIGNMENT_APPLIED` 同样进入准备状态；地图就绪后保持 `PREPARING_LOCAL_MAP`、设置内部 `local_map_ready_` 并开始连续 Scan 验证，`DIRECT_POSE_VALIDATED` 只能从“RViz 来源且地图已就绪”的准备状态进入有效定位。`LOCAL_MAP_FAILED` 回到 `LOCAL_ODOMETRY` 并请求清理，`MAP_GAP_TIMEOUT` 从跟踪态直接进入 `LOST`。

- [ ] **Step 4: 运行状态机测试 GREEN**

Run: `catkin_make run_tests_voxel_slam_gtest_localization_state_test`

Expected: PASS，原有非法事件和连续坏帧测试继续通过。

- [ ] **Step 5: 提交**

```bash
git add VoxelSLAM/src/localization_state.hpp VoxelSLAM/src/localization_state.cpp VoxelSLAM/test/localization_state_test.cpp
git commit -m "feat: add spatial map preparation state"
```

### Task 7: 将空间块接入建图导出和定位 IESKF

**Files:**
- Modify: `VoxelSLAM/src/voxelslam.cpp`
- Modify: `VoxelSLAM/src/prior_map_tracker.hpp`
- Modify: `VoxelSLAM/src/prior_map_tracker.cpp`
- Modify: `VoxelSLAM/config/localization_mid360.yaml`
- Modify: `VoxelSLAM/config/mid360.yaml`
- Modify: `VoxelSLAM/launch/vxlm_localization_mid360.launch`
- Modify: `VoxelSLAM/CMakeLists.txt`
- Test: all existing and new C++ tests

**Interfaces:**
- Consumes: v2 `LoadedMapPackage`、`SpatialMapManager`、状态机和 `StaticMapBlock::match()`。
- Produces: 建图 v2 导出、定位异步预加载、静态地图 IESKF 查询和地图缺口保护。

- [ ] **Step 1: 添加集成结构检查测试/断言并确认旧路径仍存在**

在 `voxelslam.cpp` 接线前，先为共用 IESKF 数学函数建立可测试匹配结果接口：

```cpp
struct PointPlaneObservation {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double sigma_d = 0.0;
};
```

将 IESKF 数学循环提取为以匹配回调为模板参数的私有函数；原动态 `OctoTree` 和新静态块分别提供回调。先增加编译测试，确认两个回调都能实例化且相同观测产生相同状态增量。

- [ ] **Step 2: 运行测试确认 RED**

Run: `catkin_make run_tests_voxel_slam_gtest_spatial_map_builder_test`

Expected: FAIL，集成观察接口或静态快照匹配入口尚未接入。

- [ ] **Step 3: 接入参数和地图包 v2**

建图模式从 `mid360.yaml` 读取并校验地图格式参数：

```yaml
MapBlock:
  size_x: 20.0
  size_y: 20.0
  size_z: 10.0
  fusion_leaf_size: 0.1
  root_voxel_size: 1.0

Localization:
  spatial_map:
    preload_radius: 15.0
    load_radius: 30.0
    unload_radius: 40.0
    prefetch_time_sec: 2.0
    map_gap_timeout_sec: 1.0
    preload_timeout_sec: 10.0
    max_background_loads: 2
```

建图导出把最终参数写入 `MapPackageOptions::spatial_map`。定位模式只从 v2 清单读取块尺寸、融合分辨率、根体素尺寸和静态八叉树参数；`localization_mid360.yaml` 只配置 `Localization/spatial_map` 的运行时半径、前视、超时和并发数。定位加载使用 `LoadedMapPackage::historical_map` 初始化 BTC/二维关键帧索引，并用 `spatial_catalog` 初始化块管理器。

- [ ] **Step 4: 替换连续定位局部地图路径**

删除定位主循环对 `rebuild_historical_local_map()`、`update_historical_local_map()` 和 `install_historical_cache()` 的调用。保留 `PriorMapTracker` 的二维 BTC 范围查询和最近高度查询，但连续定位不再使用三维关键帧缓存。

BTC/RViz 应用全局修正后：启动新块 Session、请求候选位姿附近核心块并触发 `ALIGNMENT_APPLIED`。主循环轮询核心就绪状态，触发 `LOCAL_MAP_READY/FAILED`。每个 Scan 取得一个快照传入静态匹配回调；快照在全部 IESKF 迭代结束前保持存活。

主循环必须单独处理 `PREPARING_LOCAL_MAP`：核心块未就绪时只保留 IMU 传播，不得落入普通本地建图分支，也不得向动态 `surf_map` 插入当前扫描。首次切入静态跟踪前显式释放临时本地里程计使用的动态 `surf_map/surf_map_slide`，不能再依赖旧 `install_historical_cache()` 的交换操作隐式释放。

- [ ] **Step 5: 实现地图缺口保护**

如果当前快照匹配质量合格，正常更新。匹配不足且状态显示核心块仍在加载，则回滚到本 Scan 的 IMU 传播状态、不累计坏帧并记录缺口起始时间；持续超过 `map_gap_timeout_sec` 发送 `MAP_GAP_TIMEOUT`。无待加载核心块时，继续使用原有 `TRACKING_BAD/NONFINITE` 逻辑。

- [ ] **Step 6: 运行全套验证**

Ubuntu:

```bash
catkin_make
catkin_make run_tests
catkin_test_results
python3 -m unittest VoxelSLAM/test/evaluate_localization_test.py
```

Expected: 编译成功，全部 GTest 和 4 个 Python 测试通过。

Windows:

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
python -m unittest VoxelSLAM/test/evaluate_localization_test.py
git diff --check
rg -n "rebuild_historical_local_map|update_historical_local_map" VoxelSLAM/src/voxelslam.cpp
```

Expected: Python 4/4 PASS；格式检查无输出；旧函数可以保留用于过渡，但定位跟踪调用点必须为零。

- [ ] **Step 7: 提交**

```bash
git add VoxelSLAM/src/voxelslam.cpp VoxelSLAM/src/prior_map_tracker.hpp VoxelSLAM/src/prior_map_tracker.cpp VoxelSLAM/config/localization_mid360.yaml VoxelSLAM/config/mid360.yaml VoxelSLAM/launch/vxlm_localization_mid360.launch VoxelSLAM/CMakeLists.txt
git commit -m "feat: localize against streamed spatial blocks"
```

### Task 8: 中文运行文档、诊断指标与最终审查

**Files:**
- Modify: `README.md`
- Modify: `VoxelSLAM/scripts/evaluate_localization.py`
- Modify: `VoxelSLAM/test/evaluate_localization_test.py`
- Modify: `docs/superpowers/specs/2026-10-06-spatial-voxel-block-map-design.md` only if implementation constraints require an explicitly documented deviation

**Interfaces:**
- Produces: v2 地图生成/转换/定位命令、状态解释、Ubuntu 验证清单和空间块性能统计。

- [ ] **Step 1: 先写性能日志解析失败测试**

```python
def test_reports_spatial_block_runtime_metrics(self):
    metrics = evaluate_lines([
        "SPATIAL_MAP active=18 pending=2 core=1 load_ms=3.2 match_ms=11.4"
    ])
    self.assertEqual(metrics.spatial_active_blocks_max, 18)
    self.assertAlmostEqual(metrics.spatial_load_ms_p95, 3.2)
    self.assertAlmostEqual(metrics.spatial_match_ms_p95, 11.4)
```

- [ ] **Step 2: 运行 Python 测试确认 RED**

Run: `python -m unittest VoxelSLAM/test/evaluate_localization_test.py`

Expected: FAIL，指标字段不存在。

- [ ] **Step 3: 实现指标解析并补充中文 README**

README 必须包含：

- 从建图模式直接导出 v2；
- 使用 `build_spatial_map --input ... --output ...` 转换 v1；
- v2 文件结构和不可修改参数；
- `PREPARING_LOCAL_MAP`、RViz 验证和 `LOST` 的含义；
- 空间块参数调节原则；
- Ubuntu `catkin_make`、测试和 rosbag 运行命令；
- Windows 只能进行哪些检查；
- 必须记录加载、匹配、总帧时延的平均值、P95、最大值和内存峰值。

- [ ] **Step 4: 运行文档和指标测试 GREEN**

Run: `python -m unittest VoxelSLAM/test/evaluate_localization_test.py`

Expected: 全部通过。

Run: `git diff --check`

Expected: 无输出。

- [ ] **Step 5: 提交**

```bash
git add README.md VoxelSLAM/scripts/evaluate_localization.py VoxelSLAM/test/evaluate_localization_test.py docs/superpowers/specs/2026-10-06-spatial-voxel-block-map-design.md
git commit -m "docs: explain spatial map localization workflow"
```

### Task 9: 全分支验证与保留旧实现清理

**Files:**
- Modify only files required by verified findings

**Interfaces:**
- Consumes: Tasks 1–8 的全部实现。
- Produces: 审查通过、工作区干净且可在 Ubuntu 复验的分支。

- [ ] **Step 1: 检查需求覆盖和禁止路径**

Run:

```bash
rg -n "install_historical_cache|rebuild_historical_local_map|update_historical_local_map" VoxelSLAM/src/voxelslam.cpp
rg -n "spatial_blocks\.idx|spatial_blocks\.bin|PREPARING_LOCAL_MAP|map_gap_timeout_sec" VoxelSLAM/src VoxelSLAM/config README.md
```

Expected: 定位连续跟踪不再调用完整重建路径；v2 文件、准备状态和缺口参数均有实现与文档。完成静态路径接线后，删除 `PriorMapTracker` 的三维索引、活动关键帧集合、合并世界点云和对应测试，仅保留二维 BTC/高度索引。

- [ ] **Step 2: 运行 Windows 可执行验证**

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
python -m unittest VoxelSLAM/test/evaluate_localization_test.py
git diff --check
git status --short
```

Expected: Python 测试通过，格式检查无输出，除实施提交外没有未跟踪或未提交文件。

- [ ] **Step 3: 记录 Ubuntu 必跑命令**

```bash
catkin_make
catkin_make run_tests
catkin_test_results
rosrun voxel_slam build_spatial_map --input /path/v1 --output /path/v2
roslaunch voxel_slam vxlm_localization_mid360.launch map_path:=/path/v2
```

不在 Windows 声称这些命令通过。将未验证项写入最终交付说明。

- [ ] **Step 4: 执行全分支代码审查并修复 Critical/Important 问题**

审查范围从 `localization-v1-baseline` 到 `HEAD`，重点检查二进制边界、整数溢出、线程退出、快照生命周期、Session 代数、地图损坏处理和原建图路径回归。

- [ ] **Step 5: 最终提交（仅存在审查修复时）**

```bash
git add -u
git commit -m "fix: harden spatial map localization"
```
