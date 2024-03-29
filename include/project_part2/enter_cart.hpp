#ifndef PROJECT_PART2_ENTER_CART_HPP_
#define PROJECT_PART2_ENTER_CART_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace project_part2 {

class EnterCart : public rclcpp::Node {
public:
  explicit EnterCart();

  static constexpr float kLinearVelocity = 0.08f;
  static constexpr float kLeftAngularVelocity = 0.1f;
  static constexpr float kRightAngularVelocity = -kLeftAngularVelocity;
  static constexpr float kTurnFuzz = 0.2f; // Precision of turn +-
  static constexpr float kTranslateFuzz = 0.05f;

  // when tf x equals this, stop relying on the cart tf
  static constexpr float kSwitchToOdomX = -0.4f;
  // How deep inside cart?
  static constexpr float kCartMinDepthX = 0.4f;

  static const float kPiOverTwo;

  static const std::string kScanTopic;
  static const std::string kCmdVelTopic;
  static const std::string kOdomTopic;

  static const std::string kParentTfFrame;
  static const std::string kChildTfFrame;

  enum class EnterCartState {
    initialize,
    align_with_cart_orientation01,
    align_perpindicular_to_cart_orientation,
    move_in_front_of_cart,    
    align_with_cart_orientation02,
    move_into_cart,
    move_to_cart_center,
    ready_to_attach
  };

private:
  
  bool is_complete_ = false;
  EnterCartState state_;
  geometry_msgs::msg::Vector3
      goal_turn_v3_; // Goal: odom pose z rotation when turning toward cart

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Issue cmd_vel commands when this timer fires 
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_ptr_;
  void cmd_vel_timer_callback();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message);

  // Get tf's when this timer fires
  rclcpp::TimerBase::SharedPtr tf_timer_ptr_;
  void tf_timer_callback();

  geometry_msgs::msg::TransformStamped transform_stamped_;
  nav_msgs::msg::Odometry::SharedPtr odom_message_;
  float goal_odom_y_; // Used only when entering the cart
  //float goal_euler_z_; // only used when aligning perpindicular to the cart
  bool have_received_tf_ = false;
  bool have_received_odom_ = false;
  // Do we need to move toward the back wall or the front wall in order to align with 
  bool is_move_toward_back_wall_;
  float goal_odom_x_to_align_with_cart_;
  float goal_odom_y_to_enter_cart_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  std::shared_ptr<geometry_msgs::msg::Twist> twist_;

  void stop();
  void turn();
  void forward();
  void backward();
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