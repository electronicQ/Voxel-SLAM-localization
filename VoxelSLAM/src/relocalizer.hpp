#pragma once

#include "BTC.h"
#include "prior_map_tracker.hpp"

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

double next_btc_radius(double current, double maximum);

std::vector<double> make_btc_radius_schedule(double initial, double maximum,
                                             bool expand_on_failure);

struct BtcRelocalizationOptions {
  bool use_prior_position = false;
  Eigen::Vector2d prior_xy = Eigen::Vector2d::Zero();
  double initial_radius = 30.0;
  bool expand_on_failure = true;
  double maximum_radius = 100.0;
  bool fallback_to_global = true;
};

struct BtcSearchMatch {
  int frame_id = -1;
  double score = 0.0;
  Eigen::Matrix3d R_HC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_HC = Eigen::Vector3d::Zero();
};

struct RelocalizationResult {
  bool success = false;
  int btc_frame_id = -1;
  int anchor_keyframe_id = -1;
  double score = 0.0;
  Eigen::Matrix3d R_HC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_HC = Eigen::Vector3d::Zero();
  bool icp_observable = false;
  double icp_observability = 0.0;
  std::vector<double> attempted_radii;
  bool global_fallback_tried = false;
  std::string failure_reason;
};

using BtcSearchFunction = std::function<bool(
    const std::unordered_set<int> &allowed_frame_ids, BtcSearchMatch &match,
    std::string &failure_reason)>;

using IcpVerificationFunction = std::function<bool(
    int btc_frame_id, Eigen::Matrix3d &R_HC, Eigen::Vector3d &t_HC,
    double &observability, std::string &failure_reason)>;

class BtcRelocalizer {
 public:
  bool initialize(const SavedMap &map, const PriorMapTracker &tracker,
                  std::string &error);

  RelocalizationResult relocalize(
      const BtcRelocalizationOptions &options,
      const BtcSearchFunction &btc_search,
      const IcpVerificationFunction &icp_verify) const;

 private:
  struct State {
    const SavedMap *map = nullptr;
    const PriorMapTracker *tracker = nullptr;
    std::unordered_map<int, int> anchor_to_btc;
    std::unordered_map<int, int> btc_to_anchor;
    std::unordered_set<int> all_btc_frames;
  };

  std::shared_ptr<State> state_;
};

// The returned callback borrows all three arguments and must be used only
// while they remain alive. Every call uses the cross-session filtered overload.
BtcSearchFunction make_stdesc_btc_search(
    STDescManager &manager, std::vector<STD> &current_descriptors,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &current_planes);
