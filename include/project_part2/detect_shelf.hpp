#ifndef PROJECT_PART2_DETECT_SHELF_HPP_
#define PROJECT_PART2_DETECT_SHELF_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <climits>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace project_part2 {

class DetectShelf : public rclcpp::Node {
public:
  explicit DetectShelf();

  static geometry_msgs::msg::Vector3 eulerFromQuaternion(const tf2::Quaternion q);

  static constexpr double kPi = 3.141592653589793238463;
  static const std::string kScanTopic;
  static constexpr int kPlateIntensity = 8000;
  // Intense readings in order to be able to detect a plate
  static constexpr int kPlateDetectionFailureThreshold = 6;
  static constexpr double kHalfPlateGap = 0.3;
  static const std::string kCartFrame;
  static const std::string kRobotLaserBaseLink;
  static constexpr double kRangeMax = 20.0;

  static std::pair<float, float> yawAndDistanceToRosXY(const float yaw, float distance);
  static float surfaceNormal(const double x1, const double y1, const double x2, const double y2);

private: 
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::TimerBase::SharedPtr timer_ptr_;
  void timer_callback();

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
  
  unsigned long getAverageHighIntensityIndex(sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  static constexpr int kIndexFailureValue = INT_MAX;
  
  static double magnitude(geometry_msgs::msg::Vector3 v3);
};
} // namespace project_part2
#endif // PROJECT_PART2_DETECT_SHELF_HPP_