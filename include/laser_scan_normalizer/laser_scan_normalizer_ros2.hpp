// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace laser_scan_normalizer {

/**
 * @brief ROS2 Component for LaserScan normalization
 */
class LaserScanNormalizerROS2 : public rclcpp::Node {
public:
  explicit LaserScanNormalizerROS2(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LaserScanNormalizerROS2() override = default;

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
};

}  // namespace laser_scan_normalizer
