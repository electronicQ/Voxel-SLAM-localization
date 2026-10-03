#include "relocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <new>
#include <utility>

namespace {

bool finiteTransform(const Eigen::Matrix3d &rotation,
                     const Eigen::Vector3d &translation) {
  return rotation.array().isFinite().all() &&
         translation.array().isFinite().all();
}

bool tryCandidateSet(
    const std::unordered_set<int> &allowed,
    const std::unordered_map<int, int> &btc_to_anchor,
    const BtcSearchFunction &btc_search,
    const IcpVerificationFunction &icp_verify,
    RelocalizationResult &result) {
  if (allowed.empty()) {
    result.failure_reason = "BTC candidate set is empty";
    return false;
  }

  BtcSearchMatch match;
  std::string reason;
  if (!btc_search(allowed, match, reason)) {
    result.failure_reason = reason.empty() ? "BTC search found no candidate"
                                           : reason;
    return false;
  }
  const auto anchor = btc_to_anchor.find(match.frame_id);
  if (anchor == btc_to_anchor.end()) {
    result.failure_reason = "BTC search returned an unknown frame ID";
    return false;
  }
  if (allowed.find(match.frame_id) == allowed.end()) {
    result.failure_reason = "BTC search returned a frame outside the allowed set";
    return false;
  }
  if (!std::isfinite(match.score) ||
      !finiteTransform(match.R_HC, match.t_HC)) {
    result.failure_reason = "BTC search returned a non-finite result";
    return false;
  }
  if (!icp_verify) {
    result.failure_reason = "ICP verification callback is unavailable";
    return false;
  }

  Eigen::Matrix3d refined_rotation = match.R_HC;
  Eigen::Vector3d refined_translation = match.t_HC;
  double observability = 0.0;
  reason.clear();
  if (!icp_verify(match.frame_id, refined_rotation, refined_translation,
                  observability, reason)) {
    result.failure_reason =
        reason.empty() ? "ICP verification rejected BTC candidate" : reason;
    return false;
  }
  if (!std::isfinite(observability) ||
      !finiteTransform(refined_rotation, refined_translation)) {
    result.failure_reason = "ICP verification returned a non-finite result";
    return false;
  }

  result.success = true;
  result.btc_frame_id = match.frame_id;
  result.anchor_keyframe_id = anchor->second;
  result.score = match.score;
  result.R_HC = refined_rotation;
  result.t_HC = refined_translation;
  result.icp_observable = true;
  result.icp_observability = observability;
  result.failure_reason.clear();
  return true;
}

}  // namespace

double next_btc_radius(double current, double maximum) {
  if (!std::isfinite(current) || !std::isfinite(maximum) || current <= 0.0 ||
      maximum < current) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::min(current * 2.0, maximum);
}

std::vector<double> make_btc_radius_schedule(double initial, double maximum,
                                             bool expand_on_failure) {
  std::vector<double> radii;
  if (!std::isfinite(initial) || !std::isfinite(maximum) || initial <= 0.0 ||
      maximum < initial) {
    return radii;
  }

  radii.push_back(initial);
  if (!expand_on_failure) {
    return radii;
  }
  while (radii.back() < maximum) {
    const double next = next_btc_radius(radii.back(), maximum);
    if (!std::isfinite(next) || next <= radii.back()) {
      break;
    }
    radii.push_back(next);
  }
  return radii;
}

bool BtcRelocalizer::initialize(const SavedMap &map,
                                const PriorMapTracker &tracker,
                                std::string &error) {
  error.clear();
  if (tracker.source_map() != &map) {
    error = "prior-map tracker is not indexed from the supplied saved map";
    return false;
  }
  if (map.btc_frames.empty()) {
    error = "saved map contains no BTC frames";
    return false;
  }

  try {
    std::shared_ptr<State> candidate(new State());
    candidate->map = &map;
    candidate->tracker = &tracker;
    std::unordered_set<int> keyframe_ids;
    keyframe_ids.reserve(map.keyframes.size());
    for (const SavedKeyframe &keyframe : map.keyframes) {
      keyframe_ids.insert(keyframe.id);
    }
    candidate->anchor_to_btc.reserve(map.btc_frames.size());
    candidate->btc_to_anchor.reserve(map.btc_frames.size());
    candidate->all_btc_frames.reserve(map.btc_frames.size());
    for (const SavedBtcFrame &frame : map.btc_frames) {
      if (frame.frame_id < 0 || frame.anchor_keyframe_id < 0) {
        error = "BTC frame and anchor IDs must be non-negative";
        return false;
      }
      if (keyframe_ids.find(frame.anchor_keyframe_id) == keyframe_ids.end()) {
        error = "BTC frame references an unknown anchor keyframe";
        return false;
      }
      if (!frame.plane_cloud) {
        error = "BTC frame plane cloud is null";
        return false;
      }
      if (!candidate->all_btc_frames.insert(frame.frame_id).second) {
        error = "saved map contains a duplicate BTC frame ID";
        return false;
      }
      if (!candidate->anchor_to_btc
               .emplace(frame.anchor_keyframe_id, frame.frame_id)
               .second) {
        error = "saved map contains a duplicate BTC anchor mapping";
        return false;
      }
      candidate->btc_to_anchor.emplace(frame.frame_id,
                                       frame.anchor_keyframe_id);
    }
    state_ = candidate;
    return true;
  } catch (const std::bad_alloc &) {
    error = "out of memory while building BTC anchor mappings";
    return false;
  } catch (const std::exception &exception) {
    error = std::string("failed to build BTC anchor mappings: ") +
            exception.what();
    return false;
  } catch (...) {
    error = "unknown failure while building BTC anchor mappings";
    return false;
  }
}

RelocalizationResult BtcRelocalizer::relocalize(
    const BtcRelocalizationOptions &options,
    const BtcSearchFunction &btc_search,
    const IcpVerificationFunction &icp_verify) const {
  RelocalizationResult result;
  if (!state_) {
    result.failure_reason = "BTC relocalizer is not initialized";
    return result;
  }
  if (!btc_search) {
    result.failure_reason = "BTC search callback is unavailable";
    return result;
  }

  try {
    if (!options.use_prior_position) {
      result.global_fallback_tried = true;
      tryCandidateSet(state_->all_btc_frames, state_->btc_to_anchor, btc_search,
                      icp_verify, result);
      return result;
    }
    if (!options.prior_xy.array().isFinite().all()) {
      result.failure_reason = "BTC prior XY position must be finite";
      return result;
    }
    const std::vector<double> radii = make_btc_radius_schedule(
        options.initial_radius, options.maximum_radius,
        options.expand_on_failure);
    if (radii.empty()) {
      result.failure_reason = "BTC radius configuration is invalid";
      return result;
    }

    for (double radius : radii) {
      result.attempted_radii.push_back(radius);
      const std::vector<int> anchors =
          state_->tracker->radius_search_xy(options.prior_xy, radius);
      std::unordered_set<int> allowed;
      allowed.reserve(anchors.size());
      for (int anchor_id : anchors) {
        const auto mapping = state_->anchor_to_btc.find(anchor_id);
        if (mapping != state_->anchor_to_btc.end()) {
          allowed.insert(mapping->second);
        }
      }
      if (tryCandidateSet(allowed, state_->btc_to_anchor, btc_search,
                          icp_verify, result)) {
        return result;
      }
    }

    if (options.fallback_to_global) {
      result.global_fallback_tried = true;
      tryCandidateSet(state_->all_btc_frames, state_->btc_to_anchor, btc_search,
                      icp_verify, result);
    }
    return result;
  } catch (const std::bad_alloc &) {
    result.success = false;
    result.failure_reason = "out of memory during BTC relocalization";
    return result;
  } catch (const std::exception &exception) {
    result.success = false;
    result.failure_reason =
        std::string("BTC relocalization callback failed: ") + exception.what();
    return result;
  } catch (...) {
    result.success = false;
    result.failure_reason = "unknown failure during BTC relocalization";
    return result;
  }
}

BtcSearchFunction make_stdesc_btc_search(
    STDescManager &manager, std::vector<STD> &current_descriptors,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &current_planes) {
  return [&manager, &current_descriptors, &current_planes](
             const std::unordered_set<int> &allowed, BtcSearchMatch &match,
             std::string &failure_reason) {
    if (!current_planes) {
      failure_reason = "current BTC plane cloud is null";
      return false;
    }
    std::pair<int, double> loop_result(-1, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    std::vector<std::pair<STD, STD>> loop_pairs;
    manager.SearchLoop(current_descriptors, loop_result, loop_transform,
                       loop_pairs, current_planes, allowed);
    if (loop_result.first < 0) {
      failure_reason = "BTC search found no verified candidate";
      return false;
    }
    match.frame_id = loop_result.first;
    match.score = loop_result.second;
    match.t_HC = loop_transform.first;
    match.R_HC = loop_transform.second;
    failure_reason.clear();
    return true;
  };
}
