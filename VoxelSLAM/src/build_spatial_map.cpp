#include "map_package.hpp"
#include "spatial_map_io.hpp"

#include <boost/filesystem.hpp>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

namespace fs = boost::filesystem;

bool parseDouble(const char *text, double &value) {
  char *end = nullptr;
  value = std::strtod(text, &end);
  return end != text && end != nullptr && *end == '\0' &&
         std::isfinite(value);
}

bool parseInt(const char *text, int &value) {
  char *end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text || end == nullptr || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool isPathPrefix(const fs::path &prefix, const fs::path &path) {
  auto prefix_it = prefix.begin();
  auto path_it = path.begin();
  for (; prefix_it != prefix.end(); ++prefix_it, ++path_it) {
    if (path_it == path.end() || *prefix_it != *path_it) {
      return false;
    }
  }
  return true;
}

void usage() {
  std::cerr
      << "用法: build_spatial_map --input <v1地图包> --output <v2地图包> "
         "[--size-x m] [--size-y m] [--size-z m] "
         "[--fusion-leaf m] [--root-voxel m] [--max-layer n] "
         "[--min-eigen v] [--save-display-cloud 0|1]\n";
}

}  // namespace

int main(int argc, char **argv) {
  std::string input;
  std::string output;
  SpatialMapConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "--help" || argument == "-h") {
      usage();
      return 0;
    }
    if (i + 1 >= argc) {
      usage();
      return 2;
    }
    const char *value = argv[++i];
    bool valid = true;
    if (argument == "--input") {
      input = value;
    } else if (argument == "--output") {
      output = value;
    } else if (argument == "--size-x") {
      valid = parseDouble(value, config.size_x);
    } else if (argument == "--size-y") {
      valid = parseDouble(value, config.size_y);
    } else if (argument == "--size-z") {
      valid = parseDouble(value, config.size_z);
    } else if (argument == "--fusion-leaf") {
      valid = parseDouble(value, config.fusion_leaf_size);
    } else if (argument == "--root-voxel") {
      valid = parseDouble(value, config.root_voxel_size);
    } else if (argument == "--max-layer") {
      valid = parseInt(value, config.max_layer);
    } else if (argument == "--min-eigen") {
      valid = parseDouble(value, config.min_eigen_value);
    } else if (argument == "--save-display-cloud") {
      int enabled = 0;
      valid = parseInt(value, enabled) && (enabled == 0 || enabled == 1);
      config.save_display_cloud = enabled != 0;
    } else {
      std::cerr << "未知参数: " << argument << '\n';
      usage();
      return 2;
    }
    if (!valid) {
      std::cerr << "无效参数值: " << argument << '=' << value << '\n';
      return 2;
    }
  }
  if (input.empty() || output.empty()) {
    usage();
    return 2;
  }

  try {
    const fs::path input_path = fs::canonical(fs::path(input));
    const fs::path output_path = fs::absolute(fs::path(output)).lexically_normal();
    if (input_path == output_path || isPathPrefix(input_path, output_path) ||
        isPathPrefix(output_path, input_path)) {
      std::cerr << "输入和输出目录不能相同或互相嵌套\n";
      return 2;
    }
    std::string config_error;
    if (!config.validate(config_error)) {
      std::cerr << "空间地图参数无效: " << config_error << '\n';
      return 2;
    }
    LoadedLegacyMapPackage legacy;
    std::string error;
    if (!MapPackageIO::load_legacy_v1(input_path.string(), legacy, error)) {
      std::cerr << "读取 v1 地图失败: " << error << '\n';
      return 1;
    }
    if (!MapPackageIO::save_converted_legacy(
            output_path.string(), legacy, config, error)) {
      std::cerr << "写入 v2 地图失败: " << error << '\n';
      return 1;
    }
    SpatialMapCatalog catalog;
    if (!SpatialMapIO::load_catalog(output_path.string(), catalog, error) ||
        !SpatialMapIO::validate_all_blocks(output_path.string(), catalog,
                                           error)) {
      std::cerr << "v2 空间块验证失败: " << error << '\n';
      boost::system::error_code ignored;
      fs::remove_all(output_path, ignored);
      return 1;
    }
    std::cout << "转换完成: " << output_path.string() << '\n';
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "转换失败: " << exception.what() << '\n';
    return 1;
  }
}
