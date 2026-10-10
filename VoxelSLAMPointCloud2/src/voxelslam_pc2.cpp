#include "voxelslam_pc2.hpp"

#include <sstream>

#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace voxelslam_pointcloud2 {

PointCloud2Display::PointCloud2Display()
  : point_cloud_common_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this)) {}

PointCloud2Display::~PointCloud2Display()
{
  unsubscribe();
  point_cloud_common_.reset();
}

void PointCloud2Display::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void PointCloud2Display::processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (!cloud) return;
  const int32_t xi = rviz_default_plugins::findChannelIndex(cloud, "x");
  const int32_t yi = rviz_default_plugins::findChannelIndex(cloud, "y");
  const int32_t zi = rviz_default_plugins::findChannelIndex(cloud, "z");
  if (xi < 0 || yi < 0 || zi < 0) return;

  const uint32_t point_step = cloud->point_step;
  const size_t point_count = static_cast<size_t>(cloud->width) * cloud->height;
  if (point_count * point_step != cloud->data.size()) {
    setStatusStd(rviz_common::properties::StatusProperty::Error, "Message",
                 "PointCloud2 data size does not match dimensions.");
    return;
  }

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
  filtered->data.clear();
  filtered->data.reserve(cloud->data.size());
  for (size_t index = 0; index < point_count; ++index) {
    const auto *point = cloud->data.data() + index * point_step;
    const float x = *reinterpret_cast<const float *>(point + xoff);
    const float y = *reinterpret_cast<const float *>(point + yoff);
    const float z = *reinterpret_cast<const float *>(point + zoff);
    if (rviz_common::validateFloats(x) && rviz_common::validateFloats(y) &&
        rviz_common::validateFloats(z)) {
      filtered->data.insert(filtered->data.end(), point, point + point_step);
    }
  }
  filtered->height = 1;
  filtered->width = static_cast<uint32_t>(filtered->data.size() / point_step);
  filtered->row_step = filtered->width * point_step;
  if (filtered->width > 0) point_cloud_common_->addMessage(filtered);
  else point_cloud_common_->reset();
}

void PointCloud2Display::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void PointCloud2Display::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

}  // namespace voxelslam_pointcloud2

PLUGINLIB_EXPORT_CLASS(voxelslam_pointcloud2::PointCloud2Display, rviz_common::Display)
