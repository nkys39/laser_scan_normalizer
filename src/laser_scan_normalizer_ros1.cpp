// SPDX-License-Identifier: MIT

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define ROS1
#include <laser_scan_normalizer/laser_scan_normalizer.hpp>

namespace laser_scan_normalizer {

class LaserScanNormalizerROS1 {
public:
  LaserScanNormalizerROS1() : nh_(), pnh_("~") {
    // Load parameters
    bool enable_resampling = pnh_.param<bool>("enable_resampling", true);
    std::string resampling_method = pnh_.param<std::string>("resampling_method", "xy");
    double dthreS = pnh_.param<double>("resampler_distance_threshold", 0.05);
    double dthreL = pnh_.param<double>("resampler_length_threshold", 0.25);

    // Configure processor
    processor_.setEnableResampling(enable_resampling);
    processor_.setResamplingMethod(resampling_method);
    if (enable_resampling) {
      processor_.setResamplerParameters(dthreS, dthreL);
      ROS_INFO("Resampling enabled: method=%s, dthreS=%.3f, dthreL=%.3f",
               resampling_method.c_str(), dthreS, dthreL);
    } else {
      ROS_INFO("Resampling disabled");
    }

    // Subscribe to input scan
    sub_scan_ = nh_.subscribe<sensor_msgs::LaserScan>(
      "scan", 10, &LaserScanNormalizerROS1::scan_callback, this);

    // Publish processed scan
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan_normalized", 10);

    ROS_INFO("LaserScanNormalizer node initialized");
  }

private:
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    sensor_msgs::LaserScan output = processor_.process(*msg);
    pub_scan_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_scan_;

  LaserScanProcessor processor_;
};

}  // namespace laser_scan_normalizer

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_normalizer_node");
  laser_scan_normalizer::LaserScanNormalizerROS1 node;
  ros::spin();
  return 0;
}
