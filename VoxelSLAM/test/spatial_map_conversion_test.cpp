#include "map_package.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <sstream>
#include <string>

namespace {

namespace fs = boost::filesystem;

class TemporaryConversionPaths {
 public:
  TemporaryConversionPaths()
      : root_(fs::temp_directory_path() /
              fs::unique_path("voxel-slam-convert-%%%%-%%%%-%%%%")),
        v1_(root_ / "legacy-v1"), v2_(root_ / "converted-v2") {
    fs::create_directories(root_);
  }
  ~TemporaryConversionPaths() {
    boost::system::error_code ignored;
    fs::remove_all(root_, ignored);
  }
  const fs::path &v1() const { return v1_; }
  const fs::path &v2() const { return v2_; }

 private:
  fs::path root_;
  fs::path v1_;
  fs::path v2_;
};

SavedMap makeMap() {
  SavedMap map;
  SavedKeyframe frame;
  frame.id = 0;
  frame.pose.stamp = 1.25;
  frame.pose.t = Eigen::Vector3d(2.0, -1.0, 0.5);
  frame.cloud.reset(new pcl::PointCloud<PointType>());
  for (int i = 0; i < 8; ++i) {
    PointType point;
    point.x = static_cast<float>(i % 4) * 0.1F;
    point.y = static_cast<float>(i / 4) * 0.1F;
    point.z = 0.0F;
    point.intensity = static_cast<float>(i);
    point.normal_x = 0.001F;
    point.normal_y = 0.001F;
    point.normal_z = 0.001F;
    point.curvature = 0.0F;
    frame.cloud->push_back(point);
  }
  frame.cloud->width = static_cast<std::uint32_t>(frame.cloud->size());
  frame.cloud->height = 1;
  frame.cloud->is_dense = true;
  map.keyframes.push_back(frame);
  return map;
}

void downgradeManifestToV1(const fs::path &path) {
  std::ifstream input(path.string());
  ASSERT_TRUE(input.good());
  std::ostringstream rewritten;
  std::string line;
  while (std::getline(input, line)) {
    if (line.compare(0, 8, "version:") == 0) {
      rewritten << "version: 1\n";
    } else if (line.compare(0, 8, "spatial_") != 0) {
      rewritten << line << '\n';
    }
  }
  input.close();
  std::ofstream output(path.string(), std::ios::trunc);
  ASSERT_TRUE(output.good());
  output << rewritten.str();
  ASSERT_TRUE(output.good());
}

TEST(SpatialMapConversion, LegacyLoaderAndConverterProduceValidatedV2) {
  TemporaryConversionPaths paths;
  MapPackageOptions original_options;
  original_options.spatial_map.min_points = {{2, 2, 2, 2}};
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(paths.v1().string(), makeMap(),
                                 original_options, error)) << error;
  downgradeManifestToV1(paths.v1() / "manifest.yaml");
  ASSERT_TRUE(fs::remove(paths.v1() / "spatial_blocks.idx"));
  ASSERT_TRUE(fs::remove(paths.v1() / "spatial_blocks.bin"));

  LoadedMapPackage runtime_rejected;
  EXPECT_FALSE(MapPackageIO::load(paths.v1().string(), runtime_rejected,
                                  original_options, error));
  EXPECT_NE(error.find("build_spatial_map"), std::string::npos) << error;

  LoadedLegacyMapPackage legacy;
  ASSERT_TRUE(MapPackageIO::load_legacy_v1(paths.v1().string(), legacy,
                                           error)) << error;
  EXPECT_EQ(legacy.historical_map.keyframes.size(), 1U);
  EXPECT_EQ(legacy.btc_config_fingerprint,
            btc_config_fingerprint(original_options.btc_config));

  SpatialMapConfig converted_config;
  converted_config.size_x = 10.0;
  converted_config.size_y = 10.0;
  converted_config.size_z = 5.0;
  converted_config.min_points = {{2, 2, 2, 2}};
  ASSERT_TRUE(MapPackageIO::save_converted_legacy(
      paths.v2().string(), legacy, converted_config, error)) << error;

  MapPackageOptions runtime_options = original_options;
  runtime_options.spatial_map = SpatialMapConfig();
  LoadedMapPackage converted;
  ASSERT_TRUE(MapPackageIO::load(paths.v2().string(), converted,
                                 runtime_options, error)) << error;
  EXPECT_DOUBLE_EQ(converted.spatial_catalog.config.size_x, 10.0);
  EXPECT_EQ(converted.historical_map.keyframes.size(), 1U);
  EXPECT_TRUE(SpatialMapIO::validate_all_blocks(
      paths.v2().string(), converted.spatial_catalog, error)) << error;
}

TEST(SpatialMapConversion, LegacyLoaderPreservesCallerOnV2Input) {
  TemporaryConversionPaths paths;
  MapPackageOptions options;
  std::string error;
  ASSERT_TRUE(MapPackageIO::save(paths.v1().string(), makeMap(), options,
                                 error)) << error;
  LoadedLegacyMapPackage legacy;
  legacy.btc_config_fingerprint = "sentinel";
  EXPECT_FALSE(MapPackageIO::load_legacy_v1(paths.v1().string(), legacy,
                                            error));
  EXPECT_EQ(legacy.btc_config_fingerprint, "sentinel");
}

}  // namespace
