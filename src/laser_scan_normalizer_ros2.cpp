// SPDX-License-Identifier: MIT

#include <laser_scan_normalizer/laser_scan_normalizer_ros2.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace laser_scan_normalizer {

LaserScanNormalizerROS2::LaserScanNormalizerROS2(const rclcpp::NodeOptions& options)
: rclcpp::Node("laser_scan_normalizer", options) {
  // Declare and get parameters
  this->declare_parameter<bool>("enable_resampling", true);
  this->declare_parameter<std::string>("resampling_method", "xy");
  this->declare_parameter<double>("resampler_distance_threshold", 0.05);
  this->declare_parameter<double>("resampler_length_threshold", 0.25);

  bool enable_resampling = this->get_parameter("enable_resampling").as_bool();
  std::string resampling_method = this->get_parameter("resampling_method").as_string();
  double dthreS = this->get_parameter("resampler_distance_threshold").as_double();
  double dthreL = this->get_parameter("resampler_length_threshold").as_double();

  // Configure processor
  processor_.setEnableResampling(enable_resampling);
  processor_.setResamplingMethod(resampling_method);
  if (enable_resampling) {
    processor_.setResamplerParameters(dthreS, dthreL);
    RCLCPP_INFO(this->get_logger(), "Resampling enabled: method=%s, dthreS=%.3f, dthreL=%.3f",
                resampling_method.c_str(), dthreS, dthreL);
  } else {
    RCLCPP_INFO(this->get_logger(), "Resampling disabled");
  }

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
  auto output = processor_.process(*msg);
  pub_scan_->publish(output);
}

}  // namespace laser_scan_normalizer

RCLCPP_COMPONENTS_REGISTER_NODE(laser_scan_normalizer::LaserScanNormalizerROS2)
