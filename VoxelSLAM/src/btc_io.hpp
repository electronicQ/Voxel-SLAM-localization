#pragma once

#include "map_package.hpp"

#include <string>
#include <vector>

bool save_btc_frames(const std::string &path,
                     const std::vector<SavedBtcFrame> &frames,
                     std::string &error);

bool load_btc_frames(const std::string &path,
                     std::vector<SavedBtcFrame> &frames,
                     std::string &error);

bool restore_btc_database(const std::vector<SavedBtcFrame> &frames,
                          STDescManager &manager, std::string &error);
