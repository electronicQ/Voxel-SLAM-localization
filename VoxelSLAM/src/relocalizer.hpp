#pragma once

#include <vector>

double next_btc_radius(double current, double maximum);

std::vector<double> make_btc_radius_schedule(double initial, double maximum,
                                             bool expand_on_failure);
