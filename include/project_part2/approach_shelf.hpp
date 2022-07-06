#ifndef PROJECT_PART2_PRE_APPROACH_HPP_
#define PROJECT_PART2_PRE_APPROACH_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
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

class ApproachShelf : public rclcpp::Node {
public:
  explicit ApproachShelf(
                          std::string scan_topic = "scan",
                          std::string odom_topic = "odom",
                          std::string cmd_vel_topic = "robot/cmd_vel");
  static geometry_msgs::msg::Vector3 euler_from_quaternion(tf2::Quaternion q);

  static constexpr double kPi = 3.141592653589793238463;

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

  static constexpr int kPlateIntensity = 8000;
  // Intense readings in order to be able to detect a plate
  static constexpr int kPlateDetectionFailureThreshold = 6;
  static const std::string kScanTopic;
  static const std::string kOdomFrame;
  static constexpr double kHalfPlateGap = 0.3;
  static const std::string kCartFrame;
  static const std::string kRobotLaserBaseLink;
  static constexpr double kRANGE_MAX = 20.0;

  enum class ApproachShelfState {
    forward_01,
    stop_forward_01,
    set_turn,
    turn,
    stop_turn,
    stop
  };

private:
  const std::string scan_topic;
  const std::string odom_topic;
  const std::string cmd_vel_topic;
  // Set inside the constructor and never changed, but can't
  // be declared constant because we must get the parameter is_gazebo in order
  // to set it correctly
  double linear_velocity_;
  bool is_complete_ = false;
  ApproachShelfState state_;
  geometry_msgs::msg::Vector3
      goal_turn_v3_; // Goal: odom pose z rotation when turning toward cart
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::TimerBase::SharedPtr timer_ptr_;
  void timer_callback();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
  
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Vector3 base_link_trans_;
  geometry_msgs::msg::Vector3 base_link_rot_;

  bool is_gazebo_;
  std::shared_ptr<geometry_msgs::msg::Twist> twist_;
  void stop();
  void turn();
  void forward();
  void publish_twist();
  static std::string state_string(ApproachShelfState state);
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
  void log_velocity() const;
};
} // namespace project_part2
#endif // PROJECT_PART2_PRE_APPROACH_HPP_