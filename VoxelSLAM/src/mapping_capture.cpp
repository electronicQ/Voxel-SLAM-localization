#include "mapping_capture.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <new>
#include <utility>

namespace {

bool fail(std::string &error, const std::string &message) {
  error = message;
  return false;
}

bool finiteVector(const Eigen::Vector3d &value) {
  return value.array().isFinite().all();
}

bool finiteMatrix(const Eigen::Matrix3d &value) {
  return value.array().isFinite().all();
}

bool finitePose(const SavedPose &pose) {
  return std::isfinite(pose.stamp) && finiteMatrix(pose.R) &&
         finiteVector(pose.t);
}

bool finitePoint(const PointType &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity) &&
         std::isfinite(point.normal_x) && std::isfinite(point.normal_y) &&
         std::isfinite(point.normal_z) && std::isfinite(point.curvature);
}

bool finitePlanePoint(const pcl::PointXYZINormal &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity) &&
         std::isfinite(point.normal_x) && std::isfinite(point.normal_y) &&
         std::isfinite(point.normal_z) && std::isfinite(point.curvature);
}

bool finiteDescriptor(const STD &descriptor) {
  return finiteVector(descriptor.triangle_) && finiteVector(descriptor.angle_) &&
         finiteVector(descriptor.center_) &&
         finiteVector(descriptor.binary_A_.location_) &&
         finiteVector(descriptor.binary_B_.location_) &&
         finiteVector(descriptor.binary_C_.location_);
}

bool validateExistingCapture(const SavedMap &saved_map, std::string &error) {
  if (saved_map.keyframes.size() != saved_map.btc_frames.size()) {
    return fail(error, "captured keyframe and BTC counts differ");
  }
  for (std::size_t i = 0; i < saved_map.keyframes.size(); ++i) {
    if (i > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      return fail(error, "captured keyframe ID exceeds int range");
    }
    const int expected = static_cast<int>(i);
    if (saved_map.keyframes[i].id != expected ||
        saved_map.btc_frames[i].frame_id != expected ||
        saved_map.btc_frames[i].anchor_keyframe_id != expected) {
      return fail(error, "captured keyframe/BTC IDs are not continuous");
    }
  }
  return true;
}

}  // namespace

bool captureAcceptedKeyframe(
    bool accepted, const SavedPose &pose,
    const pcl::PointCloud<PointType>::Ptr &keyframe_cloud,
    const std::vector<STD> &descriptors,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud,
    SavedMap &saved_map, int &capture_id, std::string &error) {
  error.clear();
  capture_id = -1;
  if (!accepted) {
    return true;
  }
  if (!validateExistingCapture(saved_map, error)) {
    return false;
  }
  if (saved_map.keyframes.size() >=
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return fail(error, "too many captured keyframes");
  }
  if (!keyframe_cloud) {
    return fail(error, "accepted keyframe cloud is null");
  }
  if (!plane_cloud) {
    return fail(error, "accepted BTC plane cloud is null");
  }
  if (!finitePose(pose)) {
    return fail(error, "accepted keyframe pose or timestamp is non-finite");
  }
  for (const PointType &point : keyframe_cloud->points) {
    if (!finitePoint(point)) {
      return fail(error, "accepted keyframe cloud contains a non-finite point");
    }
  }
  for (const pcl::PointXYZINormal &point : plane_cloud->points) {
    if (!finitePlanePoint(point)) {
      return fail(error, "accepted BTC plane cloud contains a non-finite point");
    }
  }
  for (const STD &descriptor : descriptors) {
    if (!finiteDescriptor(descriptor)) {
      return fail(error, "accepted BTC descriptor contains a non-finite value");
    }
  }

  try {
    const int new_id = static_cast<int>(saved_map.keyframes.size());
    SavedKeyframe saved_keyframe;
    saved_keyframe.id = new_id;
    saved_keyframe.pose = pose;
    saved_keyframe.cloud.reset(
        new pcl::PointCloud<PointType>(*keyframe_cloud));

    SavedBtcFrame saved_btc;
    saved_btc.frame_id = new_id;
    saved_btc.anchor_keyframe_id = new_id;
    saved_btc.descriptors = descriptors;
    for (STD &descriptor : saved_btc.descriptors) {
      descriptor.frame_number_ = new_id;
    }
    saved_btc.plane_cloud.reset(
        new pcl::PointCloud<pcl::PointXYZINormal>(*plane_cloud));
    saved_btc.plane_cloud->header.seq = static_cast<std::uint32_t>(new_id);

    const std::size_t old_keyframes = saved_map.keyframes.size();
    const std::size_t old_btc = saved_map.btc_frames.size();
    try {
      saved_map.keyframes.reserve(old_keyframes + 1U);
      saved_map.btc_frames.reserve(old_btc + 1U);
      saved_map.keyframes.push_back(std::move(saved_keyframe));
      saved_map.btc_frames.push_back(std::move(saved_btc));
    } catch (...) {
      saved_map.keyframes.resize(old_keyframes);
      saved_map.btc_frames.resize(old_btc);
      throw;
    }
    capture_id = new_id;
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory while capturing accepted keyframe");
  } catch (const std::exception &exception) {
    return fail(error, std::string("failed to capture accepted keyframe: ") +
                           exception.what());
  } catch (...) {
    return fail(error, "unknown failure while capturing accepted keyframe");
  }
}

bool refreshCapturedKeyframePoses(
    const std::vector<CapturedPoseUpdate> &pose_updates,
    SavedMap &saved_map, std::string &error) {
  error.clear();
  if (!validateExistingCapture(saved_map, error)) {
    return false;
  }
  if (pose_updates.size() != saved_map.keyframes.size()) {
    return fail(error, "final pose update count does not match captured map");
  }

  try {
    std::vector<SavedPose> refreshed(saved_map.keyframes.size());
    std::vector<bool> seen(saved_map.keyframes.size(), false);
    for (const CapturedPoseUpdate &update : pose_updates) {
      if (update.capture_id < 0 ||
          static_cast<std::size_t>(update.capture_id) >= refreshed.size()) {
        return fail(error, "final pose update has an out-of-range capture ID");
      }
      const std::size_t index = static_cast<std::size_t>(update.capture_id);
      if (seen[index]) {
        return fail(error, "final pose update has a duplicate capture ID");
      }
      if (!finitePose(update.pose)) {
        return fail(error, "final keyframe pose or timestamp is non-finite");
      }
      refreshed[index] = update.pose;
      seen[index] = true;
    }
    for (bool present : seen) {
      if (!present) {
        return fail(error, "final pose update is missing a capture ID");
      }
    }

    for (std::size_t i = 0; i < refreshed.size(); ++i) {
      saved_map.keyframes[i].pose = refreshed[i];
    }
    return true;
  } catch (const std::bad_alloc &) {
    return fail(error, "out of memory while refreshing final keyframe poses");
  } catch (const std::exception &exception) {
    return fail(error, std::string("failed to refresh final keyframe poses: ") +
                           exception.what());
  } catch (...) {
    return fail(error, "unknown failure while refreshing final keyframe poses");
  }
}
