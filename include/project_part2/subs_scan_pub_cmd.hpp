#ifndef COMPOSITION__PROJECT_PART2_COMPONENT_HPP_
#define COMPOSITION__PROJECT_PART2_COMPONENT_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "project_part2/visibility_control.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
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
  explicit SubsScanPubCmd(const rclcpp::NodeOptions &options,
                          std::string scan_topic = "scan",
                          std::string odom_topic = "odom",
                          std::string cmd_vel_topic = "robot/cmd_vel",
                          std::string elevator_down_topic = "elevator_down",
                          std::string elevator_up_topic = "elevator_up");
  static geometry_msgs::msg::Vector3 euler_from_quaternion(tf2::Quaternion q);

  static constexpr int kFrontScanRange = 540;
  static constexpr double kCloseWallDistance = 0.33;
  static constexpr double kCloseCartDistance = 0.33;
  static constexpr double kLinearVelocity = 0.08;
  static constexpr double kGazeboLinearVelocity = 0.16;
  static constexpr double kLeftAngularVelocity = 0.2;
  static constexpr double kRightAngularVelocity = -kLeftAngularVelocity;
  static const double kGoalAngularDisplacement;
  static constexpr double kTurnFuzz = 0.1; // Precision of turn +-
  static const std::string kIsGazeboParameter;

  enum class State {
    forward_01,
    stop_forward_01,
    set_turn,
    turn,
    stop_turn,
    forward_02,
    stop_forward_02,
    elevator_up,
    stop
  };

private:
  const std::string scan_topic;
  const std::string odom_topic;
  const std::string cmd_vel_topic;
  const std::string elevator_up_topic;
  const std::string elevator_down_topic;
  // Set inside the constructor and never changed, but can't
  // be declared constant because we must get the parameter is_gazebo in order
  // to set it correctly
  double linear_velocity_;
  State state_;
  geometry_msgs::msg::Vector3
      goal_turn_v3_; // Goal: odom pose z rotation when turning toward cart
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  rclcpp::TimerBase::SharedPtr timer_ptr_;
  void timer_callback();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_down_publisher_;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  bool is_gazebo_;
  std::mutex state_mutex_;
  std::shared_ptr<geometry_msgs::msg::Twist> twist_;
  std::shared_ptr<std_msgs::msg::Empty> empty_;
  void stop();
  void turn();
  void forward();
  void elevator_up();
  void publish_twist();
  static std::string state_string(State state);
  static double magnitude(geometry_msgs::msg::Vector3 v3);
  static bool is_stopped(geometry_msgs::msg::Vector3 v3);
  // States in which the node is waiting for the robot to slow and stop
  bool is_buffer_stop_state() const;
  // States whose status is monitored using odom
  bool is_odom_state() const;
  // States whose status must be monitored by laser scan
  bool is_scan_state() const;
  // Log state using DEBUG level
  void log_state_verbose() const;
  // Log state using INFO level
  void log_state() const;
};
} // namespace project_part2
#endif // COMPOSITION__PROJECT_PART2_COMPONENT_HPP_