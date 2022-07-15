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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace project_part2 {

class EnterCart : public rclcpp::Node {
public:
  explicit EnterCart();

  static constexpr int kFrontScanRange = 540;
  static constexpr float kCloseCartDistance = 0.33;
  static constexpr float kLinearVelocity = 0.08;
  static constexpr float kLeftAngularVelocity = 0.1;
  static constexpr float kRightAngularVelocity = -kLeftAngularVelocity;
  static constexpr float kTurnFuzz = 0.1; // Precision of turn +-
  static constexpr float kTranslateFuzz = 0.01;

  static const std::string kScanTopic;
  static const std::string kCmdVelTopic;

  static const std::string kParentTfFrame;
  static const std::string kChildTfFrame;

  enum class EnterCartState {
    initialize,
    align_with_cart_orientation,
    align_with_cart_y,
    move_into_cart,
    ready_to_attach
  };

private:
  
  bool is_complete_ = false;
  EnterCartState state_;
  geometry_msgs::msg::Vector3
      goal_turn_v3_; // Goal: odom pose z rotation when turning toward cart
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Issue cmd_vel commands when this timer fires 
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_ptr_;
  void cmd_vel_timer_callback();

  // Get tf's when this timer fires
  rclcpp::TimerBase::SharedPtr tf_timer_ptr_;
  void tf_timer_callback();

  geometry_msgs::msg::TransformStamped transform_stamped_;
  bool have_received_tf_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);

  sensor_msgs::msg::LaserScan::SharedPtr scan_message_;
  std::shared_ptr<geometry_msgs::msg::Twist> twist_;

  void stop();
  void turn();
  void strafe(bool isLeft);
  void forward();
  void publish_twist();

  static std::string state_string(EnterCartState state);
  static bool is_stopped(geometry_msgs::msg::Vector3 v3);
  
  // Log state using DEBUG level
  void log_state_verbose() const;
  // Log state using INFO level
  void log_state() const;
  void log_velocity() const;
};
} // namespace project_part2
#endif // PROJECT_PART2_ENTER_CART_HPP_