#ifndef PROJECT_PART2_ENTER_CART_HPP_
#define PROJECT_PART2_ENTER_CART_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace project_part2 {

class EnterCart : public rclcpp::Node {
public:
  explicit EnterCart();

  static constexpr int kFrontScanRange = 540;
  static constexpr double kCloseCartDistance = 0.33;
  static constexpr double kLinearVelocity = 0.08;
  static constexpr double kGazeboLinearVelocity = 0.16;
  static constexpr double kLeftAngularVelocity = 0.2;
  static constexpr double kRightAngularVelocity = -kLeftAngularVelocity;
  static constexpr double kTurnFuzz = 0.1; // Precision of turn +-
  static const std::string kIsGazeboParameter;

  static const std::string kScanTopic;
  static const std::string kCmdVelTopic;

  enum class EnterCartState {
    initialize,
    align_with_cart_orientation,
    align_with_cart_y,
    move_into_cart,
    ready_to_attach
  };

private:
  
  // Set inside the constructor and never changed, but can't
  // be declared constant because we must get the parameter is_gazebo in order
  // to set it correctly
  double linear_velocity_;
  bool is_complete_ = false;
  EnterCartState state_;
  geometry_msgs::msg::Vector3
      goal_turn_v3_; // Goal: odom pose z rotation when turning toward cart
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::TimerBase::SharedPtr timer_ptr_;
  void timer_callback();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);

  bool is_gazebo_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_message_;
  std::shared_ptr<geometry_msgs::msg::Twist> twist_;

  void stop();
  void turn();
  void strafe();
  void forward();
  void publish_twist();

  static std::string state_string(EnterCartState state);
  static double magnitude(geometry_msgs::msg::Vector3 v3);
  static bool is_stopped(geometry_msgs::msg::Vector3 v3);
  
  // Log state using DEBUG level
  void log_state_verbose() const;
  // Log state using INFO level
  void log_state() const;
  
  void log_velocity() const;
};
} // namespace project_part2
#endif // PROJECT_PART2_ENTER_CART_HPP_