// SPDX-License-Identifier: MIT

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <laser_scan_normalizer/laser_scan_normalizer_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_scan_normalizer::LaserScanNormalizerROS2>());
  rclcpp::shutdown();
  return 0;
}
