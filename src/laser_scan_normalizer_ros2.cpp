// SPDX-License-Identifier: MIT

#include <laser_scan_normalizer/laser_scan_normalizer.hpp>
#include <laser_scan_normalizer/laser_scan_normalizer_ros2.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace laser_scan_normalizer {

LaserScanNormalizerROS2::LaserScanNormalizerROS2(const rclcpp::NodeOptions& options)
: rclcpp::Node("laser_scan_normalizer", options) {
  // Subscribe to input scan
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&LaserScanNormalizerROS2::scan_callback, this, std::placeholders::_1));

  // Publish processed scan
  pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_normalized", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "LaserScanNormalizer node initialized");
}

void LaserScanNormalizerROS2::scan_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  LaserScanProcessor processor;
  auto output = processor.process(*msg);
  pub_scan_->publish(output);
}

}  // namespace laser_scan_normalizer

RCLCPP_COMPONENTS_REGISTER_NODE(laser_scan_normalizer::LaserScanNormalizerROS2)
