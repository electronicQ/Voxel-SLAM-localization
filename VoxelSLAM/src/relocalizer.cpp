#include "relocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
