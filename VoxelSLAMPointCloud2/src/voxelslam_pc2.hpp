#pragma once

#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rviz_common/message_filter_display.hpp>

namespace rviz_default_plugins { class PointCloudCommon; }

namespace voxelslam_pointcloud2 {
class PointCloud2Display : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud2>
{
public:
  PointCloud2Display();
  ~PointCloud2Display() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;
protected:
  void onInitialize() override;
  void processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) override;
private:
  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;
};
}  // namespace voxelslam_pointcloud2
