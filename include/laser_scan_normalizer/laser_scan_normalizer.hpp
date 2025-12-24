// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

#ifdef ROS1
#include <sensor_msgs/LaserScan.h>
#else
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

namespace laser_scan_normalizer {

#ifdef ROS1
using LaserScan = sensor_msgs::LaserScan;
#else
using LaserScan = sensor_msgs::msg::LaserScan;
#endif

/**
 * @brief LaserScan processing utility class
 *
 * This class provides common processing functions for LaserScan messages.
 * It can be used with both ROS1 and ROS2.
 */
class LaserScanProcessor {
public:
  LaserScanProcessor() = default;

  /**
   * @brief Process a LaserScan message
   *
   * @param input Input LaserScan message
   * @return Processed LaserScan message
   */
  LaserScan process(const LaserScan& input) const {
    LaserScan output = input;

    // Example processing: normalize range values
    // TODO: Implement your actual processing logic here
    for (size_t i = 0; i < output.ranges.size(); ++i) {
      float range = output.ranges[i];

      // Skip invalid ranges
      if (!std::isfinite(range)) {
        continue;
      }

      // Clamp to valid range
      if (range < output.range_min) {
        output.ranges[i] = output.range_min;
      } else if (range > output.range_max) {
        output.ranges[i] = output.range_max;
      }
    }

    return output;
  }
};

}  // namespace laser_scan_normalizer
