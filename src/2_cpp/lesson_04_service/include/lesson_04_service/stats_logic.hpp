#pragma once

#include <numeric> // for std::accumulate
#include <string>
#include <vector>

namespace stats_logic {

// A pure C++ struct to hold the result (No ROS dependencies)
struct StatsResult {
  double sum;
  double average;
  std::string status;
};

class Logic {
public:
  // "static" means we don't need to instantiate the class to use this function
  static StatsResult compute(const std::vector<double> &data) {
    if (data.empty()) {
      return StatsResult{0.0, 0.0, "Warning: No data provided. Returning 0."};
    }

    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    double average = sum / data.size();

    return StatsResult{sum, average, "Success"};
  }
};

} // namespace stats_logic