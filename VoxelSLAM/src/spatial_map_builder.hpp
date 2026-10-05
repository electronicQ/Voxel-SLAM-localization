#pragma once

#include "spatial_map_types.hpp"

#include <string>

class SpatialMapBuilder {
 public:
  static bool build(const SpatialMapPointVector &points,
                    const SpatialMapConfig &config,
                    SpatialMapBuildResult &result,
                    std::string &error);
};
