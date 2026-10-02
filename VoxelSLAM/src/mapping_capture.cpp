#include "mapping_capture.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <new>
#include <unordered_set>
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

bool finiteState(const IMUST &state) {
  return std::isfinite(state.t) && finiteMatrix(state.R) &&
         finiteVector(state.p);
}

SavedPose poseFromState(const IMUST &state) {
  SavedPose pose;
  pose.stamp = state.t;
  pose.R = state.R;
  pose.t = state.p;
  return pose;
}

bool validateExistingCapture(
    const SavedMap &saved_map,
    const std::vector<const Keyframe *> &final_pose_sources,
    std::string &error) {
  if (saved_map.keyframes.size() != saved_map.btc_frames.size() ||
      saved_map.keyframes.size() != final_pose_sources.size()) {
    return fail(error, "captured keyframe/BTC/source counts differ");
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
    bool accepted, const Keyframe &keyframe,
    const std::vector<STD> &descriptors,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud,
    SavedMap &saved_map, std::vector<const Keyframe *> &final_pose_sources,
    std::string &error) {
  error.clear();
  if (!accepted) {
    return true;
  }
  if (!validateExistingCapture(saved_map, final_pose_sources, error)) {
    return false;
  }
  if (saved_map.keyframes.size() >=
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return fail(error, "too many captured keyframes");
  }
  if (!keyframe.plptr) {
    return fail(error, "accepted keyframe cloud is null");
  }
  if (!plane_cloud) {
    return fail(error, "accepted BTC plane cloud is null");
  }
  if (!finiteState(keyframe.x0)) {
    return fail(error, "accepted keyframe pose or timestamp is non-finite");
  }
  for (const PointType &point : keyframe.plptr->points) {
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
  for (const Keyframe *source : final_pose_sources) {
    if (source == &keyframe) {
      return fail(error, "accepted keyframe source was already captured");
    }
  }

  try {
    const int capture_id = static_cast<int>(saved_map.keyframes.size());
    SavedKeyframe saved_keyframe;
    saved_keyframe.id = capture_id;
    saved_keyframe.pose = poseFromState(keyframe.x0);
    saved_keyframe.cloud.reset(
        new pcl::PointCloud<PointType>(*keyframe.plptr));

    SavedBtcFrame saved_btc;
    saved_btc.frame_id = capture_id;
    saved_btc.anchor_keyframe_id = capture_id;
    saved_btc.descriptors = descriptors;
    for (STD &descriptor : saved_btc.descriptors) {
      descriptor.frame_number_ = capture_id;
    }
    saved_btc.plane_cloud.reset(
        new pcl::PointCloud<pcl::PointXYZINormal>(*plane_cloud));
    saved_btc.plane_cloud->header.seq =
        static_cast<std::uint32_t>(capture_id);

    const std::size_t old_keyframes = saved_map.keyframes.size();
    const std::size_t old_btc = saved_map.btc_frames.size();
    const std::size_t old_sources = final_pose_sources.size();
    try {
      saved_map.keyframes.reserve(old_keyframes + 1U);
      saved_map.btc_frames.reserve(old_btc + 1U);
      final_pose_sources.reserve(old_sources + 1U);
      saved_map.keyframes.push_back(std::move(saved_keyframe));
      saved_map.btc_frames.push_back(std::move(saved_btc));
      final_pose_sources.push_back(&keyframe);
    } catch (...) {
      saved_map.keyframes.resize(old_keyframes);
      saved_map.btc_frames.resize(old_btc);
      final_pose_sources.resize(old_sources);
      throw;
    }
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
    const std::vector<const Keyframe *> &final_pose_sources,
    SavedMap &saved_map, std::string &error) {
  error.clear();
  if (!validateExistingCapture(saved_map, final_pose_sources, error)) {
    return false;
  }

  try {
    std::unordered_set<const Keyframe *> unique_sources;
    std::vector<SavedPose> refreshed;
    unique_sources.reserve(final_pose_sources.size());
    refreshed.reserve(final_pose_sources.size());
    for (const Keyframe *source : final_pose_sources) {
      if (source == nullptr) {
        return fail(error, "final keyframe pose source is null");
      }
      if (!unique_sources.insert(source).second) {
        return fail(error, "final keyframe pose source is duplicated");
      }
      if (!finiteState(source->x0)) {
        return fail(error, "final keyframe pose or timestamp is non-finite");
      }
      refreshed.push_back(poseFromState(source->x0));
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
