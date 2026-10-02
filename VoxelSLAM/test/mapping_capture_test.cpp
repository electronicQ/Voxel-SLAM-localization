#include "mapping_capture.hpp"
#include "voxel_map.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

namespace {

PointType point(float x, float y, float z) {
  PointType result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = 4.0F;
  result.normal_x = 1.0F;
  result.normal_y = 0.0F;
  result.normal_z = 0.0F;
  result.curvature = 0.1F;
  return result;
}

pcl::PointXYZINormal planePoint(float x) {
  pcl::PointXYZINormal result;
  result.x = x;
  result.y = x + 1.0F;
  result.z = x + 2.0F;
  result.intensity = 1.0F;
  result.normal_x = 0.0F;
  result.normal_y = 0.0F;
  result.normal_z = 1.0F;
  result.curvature = 0.01F;
  return result;
}

STD descriptor(int source_frame) {
  STD result;
  result.triangle_ = Eigen::Vector3d(1.0, 2.0, 3.0);
  result.angle_ = Eigen::Vector3d(0.1, 0.2, 0.3);
  result.center_ = Eigen::Vector3d(4.0, 5.0, 6.0);
  result.frame_number_ = source_frame;
  result.binary_A_.summary_ = 1;
  result.binary_A_.location_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  result.binary_A_.occupy_array_ = {true, false, true};
  result.binary_B_.summary_ = 2;
  result.binary_B_.location_ = Eigen::Vector3d(0.0, 1.0, 0.0);
  result.binary_B_.occupy_array_ = {false, true};
  result.binary_C_.summary_ = 3;
  result.binary_C_.location_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  result.binary_C_.occupy_array_ = {true};
  return result;
}

Keyframe makeKeyframe(int source_id, double stamp,
                      const Eigen::Vector3d &translation) {
  IMUST state;
  state.t = stamp;
  state.R = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
  state.p = translation;
  Keyframe keyframe(state);
  keyframe.id = source_id;
  keyframe.plptr->push_back(point(1.0F, 2.0F, 3.0F));
  return keyframe;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr makePlaneCloud(float seed) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr result(
      new pcl::PointCloud<pcl::PointXYZINormal>());
  result->push_back(planePoint(seed));
  return result;
}

SavedPose savedPose(const Keyframe &keyframe) {
  SavedPose result;
  result.stamp = keyframe.x0.t;
  result.R = keyframe.x0.R;
  result.t = keyframe.x0.p;
  return result;
}

TEST(MappingCapture, AcceptedKeyframeCreatesPairedContinuousRecords) {
  Keyframe first = makeKeyframe(47, 100.5, Eigen::Vector3d(1.0, 2.0, 3.0));
  Keyframe second = makeKeyframe(3, 101.5, Eigen::Vector3d(4.0, 5.0, 6.0));
  SavedMap saved;
  std::string error;
  int first_capture_id = -1;
  int second_capture_id = -1;

  ASSERT_TRUE(captureAcceptedKeyframe(
      true, savedPose(first), first.plptr, {descriptor(47)},
      makePlaneCloud(10.0F), saved, first_capture_id, error)) << error;
  ASSERT_TRUE(captureAcceptedKeyframe(
      true, savedPose(second), second.plptr, {descriptor(3)},
      makePlaneCloud(20.0F), saved, second_capture_id, error)) << error;

  ASSERT_EQ(saved.keyframes.size(), 2U);
  ASSERT_EQ(saved.btc_frames.size(), 2U);
  EXPECT_EQ(first_capture_id, 0);
  EXPECT_EQ(second_capture_id, 1);
  EXPECT_EQ(saved.keyframes[0].id, 0);
  EXPECT_EQ(saved.keyframes[1].id, 1);
  EXPECT_EQ(saved.btc_frames[1].frame_id, 1);
  EXPECT_EQ(saved.btc_frames[1].anchor_keyframe_id, 1);
  EXPECT_EQ(saved.btc_frames[1].descriptors[0].frame_number_, 1);
  EXPECT_DOUBLE_EQ(saved.keyframes[0].pose.stamp, 100.5);
  EXPECT_TRUE(saved.keyframes[1].pose.t.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
}

TEST(MappingCapture, DeepCopiesAllMutablePayloads) {
  Keyframe source = makeKeyframe(9, 12.0, Eigen::Vector3d::Zero());
  std::vector<STD> descriptors{descriptor(9)};
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr planes = makePlaneCloud(5.0F);
  SavedMap saved;
  std::string error;
  int capture_id = -1;

  ASSERT_TRUE(captureAcceptedKeyframe(
      true, savedPose(source), source.plptr, descriptors, planes, saved,
      capture_id, error)) << error;
  source.plptr->points[0].x = 99.0F;
  descriptors[0].triangle_.x() = 99.0;
  descriptors[0].binary_A_.occupy_array_[0] = false;
  planes->points[0].x = 99.0F;

  EXPECT_FLOAT_EQ(saved.keyframes[0].cloud->points[0].x, 1.0F);
  EXPECT_DOUBLE_EQ(saved.btc_frames[0].descriptors[0].triangle_.x(), 1.0);
  EXPECT_TRUE(saved.btc_frames[0]
                  .descriptors[0]
                  .binary_A_
                  .occupy_array_[0]);
  EXPECT_FLOAT_EQ(saved.btc_frames[0].plane_cloud->points[0].x, 5.0F);
}

TEST(MappingCapture, RejectedMotionIsANoOp) {
  Keyframe source = makeKeyframe(1, 1.0, Eigen::Vector3d::Zero());
  SavedMap saved;
  std::string error;
  int capture_id = 99;

  EXPECT_TRUE(captureAcceptedKeyframe(false, savedPose(source), source.plptr,
                                      {}, nullptr, saved, capture_id, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(capture_id, -1);
  EXPECT_TRUE(saved.keyframes.empty());
  EXPECT_TRUE(saved.btc_frames.empty());
}

TEST(MappingCapture, InvalidInputLeavesDestinationUnchanged) {
  Keyframe valid = makeKeyframe(1, 1.0, Eigen::Vector3d::Zero());
  SavedMap saved;
  std::string error;
  int capture_id = -1;
  ASSERT_TRUE(captureAcceptedKeyframe(
      true, savedPose(valid), valid.plptr, {descriptor(1)},
      makePlaneCloud(1.0F), saved, capture_id, error)) << error;

  const std::size_t keyframes_before = saved.keyframes.size();
  const std::size_t btc_before = saved.btc_frames.size();
  Keyframe invalid = makeKeyframe(2, 2.0, Eigen::Vector3d::Zero());
  invalid.x0.p.x() = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(captureAcceptedKeyframe(
      true, savedPose(invalid), invalid.plptr, {descriptor(2)},
      makePlaneCloud(2.0F), saved, capture_id, error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(saved.keyframes.size(), keyframes_before);
  EXPECT_EQ(saved.btc_frames.size(), btc_before);

  EXPECT_FALSE(captureAcceptedKeyframe(
      true, savedPose(valid), valid.plptr, {descriptor(1)}, nullptr, saved,
      capture_id, error));
  EXPECT_EQ(saved.keyframes.size(), keyframes_before);
  EXPECT_EQ(saved.btc_frames.size(), btc_before);
}

TEST(MappingCapture, RefreshesFinalPosesAtomically) {
  Keyframe first = makeKeyframe(8, 10.0, Eigen::Vector3d::Zero());
  Keyframe second = makeKeyframe(2, 20.0, Eigen::Vector3d::Ones());
  SavedMap saved;
  std::string error;
  int first_id = -1;
  int second_id = -1;
  ASSERT_TRUE(captureAcceptedKeyframe(
      true, savedPose(first), first.plptr, {descriptor(8)},
      makePlaneCloud(1.0F), saved, first_id, error));
  ASSERT_TRUE(captureAcceptedKeyframe(
      true, savedPose(second), second.plptr, {descriptor(2)},
      makePlaneCloud(2.0F), saved, second_id, error));

  first.x0.t = 30.0;
  first.x0.p = Eigen::Vector3d(3.0, 4.0, 5.0);
  second.x0.t = 40.0;
  second.x0.p = Eigen::Vector3d(6.0, 7.0, 8.0);
  CapturedPoseUpdate first_update;
  first_update.capture_id = first_id;
  first_update.pose = savedPose(first);
  CapturedPoseUpdate second_update;
  second_update.capture_id = second_id;
  second_update.pose = savedPose(second);

  ASSERT_TRUE(refreshCapturedKeyframePoses(
      {second_update, first_update}, saved, error)) << error;
  EXPECT_DOUBLE_EQ(saved.keyframes[0].pose.stamp, 30.0);
  EXPECT_TRUE(saved.keyframes[1].pose.t.isApprox(Eigen::Vector3d(6.0, 7.0, 8.0)));

  const SavedPose first_before = saved.keyframes[0].pose;
  const SavedPose second_before = saved.keyframes[1].pose;
  second_update.capture_id = first_id;
  EXPECT_FALSE(refreshCapturedKeyframePoses(
      {first_update, second_update}, saved, error));
  EXPECT_DOUBLE_EQ(saved.keyframes[0].pose.stamp, first_before.stamp);
  EXPECT_TRUE(saved.keyframes[0].pose.t.isApprox(first_before.t));
  EXPECT_DOUBLE_EQ(saved.keyframes[1].pose.stamp, second_before.stamp);
  EXPECT_TRUE(saved.keyframes[1].pose.t.isApprox(second_before.t));

  EXPECT_FALSE(refreshCapturedKeyframePoses({first_update}, saved, error));
  EXPECT_DOUBLE_EQ(saved.keyframes[1].pose.stamp, second_before.stamp);
}

}  // namespace
