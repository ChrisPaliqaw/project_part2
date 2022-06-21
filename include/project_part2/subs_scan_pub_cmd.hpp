#ifndef COMPOSITION__PROJECT_PART2_COMPONENT_HPP_
#define COMPOSITION__PROJECT_PART2_COMPONENT_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "project_part2/visibility_control.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace project_part2 {

class SubsScanPubCmd : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit SubsScanPubCmd(const rclcpp::NodeOptions & options,
                          std::string scan_topic = "scan",
                          std::string odom_topic = "odom",
                          std::string cmd_vel_topic = "robot/cmd_vel");
  static geometry_msgs::msg::Vector3 euler_from_quaternion(tf2::Quaternion q);

  static constexpr int kFrontScanRange = 540;
  static constexpr double kCloseWallDistance = 0.35;
  static constexpr double kLinearVelocity = 0.6;
  static constexpr double kLeftAngularVelocity = 0.2;
  static constexpr double kRightAngularVelocity = -kLeftAngularVelocity;
  static const double kGoalAngularDisplacement;
  static constexpr double kTurnFuzz = 0.1; // Precision of turn +-

  enum class State { forward_01, set_turn, turn, forward_02, stop };

private:
  const std::string scan_topic;
  const std::string odom_topic;
  const std::string cmd_vel_topic;
  State state_;
  geometry_msgs::msg::Vector3 goal_turn_v3_; // Goal: odom pose z rotation when turning toward cart
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  std::mutex state_mutex_;
  std::shared_ptr<geometry_msgs::msg::Twist> twist_;
  void stop();
  void turn();
  void forward();
  void publish_twist();
  static const std::string state_string(State state);
  void log_state_verbose() const;
  void log_state() const;
};
} // namespace project_part2
#endif // COMPOSITION__PROJECT_PART2_COMPONENT_HPP_