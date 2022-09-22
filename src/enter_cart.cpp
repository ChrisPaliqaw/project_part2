#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "../include/project_part2/enter_cart.hpp"
#include "linear_algebra_tools/linear_algebra.hpp"

using namespace std::chrono_literals;

/*
user:~$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular
parts.

Vector3  linear
Vector3  angular
*/

/*
user:~$ ros2 interface show geometry_msgs/msg/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is
applied.

float64 x
float64 y
float64 z
*/

/*
user:~/ros2_ws$ ros2 interface show geometry_msgs/msg/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
*/

using std::placeholders::_1;

namespace project_part2 {
EnterCart::EnterCart()
    : Node("enter_cart"), state_(EnterCartState::initialize) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  cmd_vel_timer_ptr_ = this->create_wall_timer(
      0.5s, std::bind(&EnterCart::cmd_vel_timer_callback, this),
      callback_group_);
  tf_timer_ptr_ = this->create_wall_timer(
      0.5s, std::bind(&EnterCart::tf_timer_callback, this), callback_group_);

  rclcpp::SubscriptionOptions odom_options;
  odom_options.callback_group = callback_group_;
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      kOdomTopic, 10, std::bind(&EnterCart::odomCallback, this, _1),
      odom_options);

  cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);

  RCLCPP_INFO_STREAM(this->get_logger(), "Create EnterCart");

  twist_ = std::make_shared<geometry_msgs::msg::Twist>();

  auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                              RCUTILS_LOG_SEVERITY_INFO);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(get_logger(), "Error setting severity: %s",
                 rcutils_get_error_string().str);
    rcutils_reset_error();
  }

  log_state();
}

void EnterCart::cmd_vel_timer_callback() {
  if (is_complete_) {
    return;
  }

  float y_translation = transform_stamped_.transform.translation.y;
  float x_translation = transform_stamped_.transform.translation.x;
  float y_odom = odom_message_->pose.pose.position.y;
  float x_odom = odom_message_->pose.pose.position.x;

  log_state_verbose();

  if (state_ == EnterCartState::initialize) {
    // When moving in front of the cart, are we trying to point toward the
    // starting wall or the opposing wall?
    if (have_received_tf_ && have_received_odom_) {
      if (!have_received_tf_) {
        RCLCPP_INFO(get_logger(), "Waiting for tf data...");
      }
      if (!have_received_odom_) {
        RCLCPP_INFO(get_logger(), "Waiting for odom data...");
      }
      state_ = EnterCartState::align_with_cart_orientation01;

      log_state();
    }
    return;
  }

  tf2::Quaternion tf2_quat_from_msg;
  tf2::fromMsg(transform_stamped_.transform.rotation, tf2_quat_from_msg);

  geometry_msgs::msg::Vector3 cart_euler =
      linear_algebra_utilities::EulerFromQuaternion(tf2_quat_from_msg);
  RCLCPP_DEBUG_STREAM(get_logger(), "Cart rotation: (" << cart_euler.x << ", "
                                                       << cart_euler.y << ", "
                                                       << cart_euler.z << ")");
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "Cart translation: ("
                          << transform_stamped_.transform.translation.x << ", "
                          << transform_stamped_.transform.translation.y << ", "
                          << transform_stamped_.transform.translation.z << ")");

  switch (state_) {
  case EnterCartState::align_with_cart_orientation01:
    if (abs(cart_euler.z) <= kTurnFuzz) {
      stop();
      state_ = EnterCartState::align_perpindicular_to_cart_orientation;
      log_state();
      // We need to plan our alignment now, since once we start turning,
      // detect_cart will lose sight of the cart and the cart's tf
      // will no longer be accurate
      is_move_toward_back_wall_ = (y_translation < 0.0f);
      RCLCPP_INFO_STREAM(get_logger(),
                         "Moving toward the back wall = "
                             << (is_move_toward_back_wall_ ? "true" : "false"));
      RCLCPP_INFO_STREAM(get_logger(),
                         "Odom rotation around z = "
                             << odom_message_->pose.pose.orientation.z);
      float goal_odom_distance_to_align_with_cart = abs(y_translation);
      // abs(cos(odom_message_->pose.pose.orientation.z) * y_translation);
      RCLCPP_INFO_STREAM(get_logger(),
                         "Lateral distance to align with cart = "
                             << goal_odom_distance_to_align_with_cart);
      goal_odom_x_to_align_with_cart_ =
          ((is_move_toward_back_wall_ ? 1.0f : -1.0f) *
           goal_odom_distance_to_align_with_cart) +
          odom_message_->pose.pose.position.x;
      RCLCPP_INFO_STREAM(get_logger(), "current odom x = " << x_odom);
      RCLCPP_INFO_STREAM(get_logger(), "goal_odom_x_to_align_with_cart = "
                                           << goal_odom_x_to_align_with_cart_);
      float goal_odom_distance_to_enter_cart =
          abs(x_translation) + kCartMinDepthX;
      RCLCPP_INFO_STREAM(get_logger(), "goal_odom_distance_to_enter_cart = "
                                           << goal_odom_distance_to_enter_cart);
      goal_odom_y_to_enter_cart_ = -(goal_odom_distance_to_enter_cart) +
                                   odom_message_->pose.pose.position.y;
      RCLCPP_INFO_STREAM(get_logger(), "current odom y = " << y_odom);
      RCLCPP_INFO_STREAM(get_logger(), "goal_odom_y_to_enter_cart_ = "
                                           << goal_odom_y_to_enter_cart_);

    } else {
      turn();
    }
    break;
  case EnterCartState::align_perpindicular_to_cart_orientation:
    if (abs(cart_euler.z - kPiOverTwo) <= kTurnFuzz) {
      stop();
      state_ = EnterCartState::move_in_front_of_cart;
      log_state();
    } else {
      turn();
    }
    break;
  case EnterCartState::move_in_front_of_cart:
    // TODO: eliminate danger of overshooting
    if (abs(x_odom - goal_odom_x_to_align_with_cart_) <= kTranslateFuzz) {
      stop();
      state_ = EnterCartState::align_with_cart_orientation02;
      log_state();
    } else if (is_move_toward_back_wall_) {
      RCLCPP_INFO(get_logger(), "Backing up...");
      backward();
    } else {
      RCLCPP_INFO(get_logger(), "Backing down...");
      forward();
    }
    break;
  case EnterCartState::align_with_cart_orientation02:

    if (abs(cart_euler.z) <= kTurnFuzz) {
      stop();
      state_ = EnterCartState::move_into_cart;
      log_state();
    } else {
      turn();
    }
    break;
  case EnterCartState::move_into_cart:
    if (y_odom <= goal_odom_y_to_enter_cart_) {
      stop();
      state_ = EnterCartState::move_to_cart_center;
      RCLCPP_INFO_STREAM(get_logger(), "goal_odom_y_ = " << goal_odom_y_);
      log_state();
    } else {
      forward();
    }
    break;
  case EnterCartState::move_to_cart_center:
    RCLCPP_DEBUG_STREAM(get_logger(), "goal_odom_y_ = " << goal_odom_y_);
    RCLCPP_DEBUG_STREAM(get_logger(), "y_odom  = " << y_odom);
    if (y_odom <= goal_odom_y_) {
      stop();
      state_ = EnterCartState::ready_to_attach;
      log_state();
      is_complete_ = true;
    } else {
      forward();
    }
    break;
  default:
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Unexpected state in cmd_vel_timer_callback(): "
                            << state_string(state_));
    break;
  }

  publish_twist();
}

void EnterCart::tf_timer_callback() {
  if (is_complete_) {
    return;
  }

  // Look up for the transformation between target_frame and turtle2 frames
  // and send velocity commands for turtle2 to reach target_frame
  try {
    transform_stamped_ = tf_buffer_->lookupTransform(
        kChildTfFrame, kParentTfFrame, tf2::TimePointZero);
    have_received_tf_ = true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_INFO_STREAM(get_logger(), "Could not transform "
                                         << kChildTfFrame << " from "
                                         << kParentTfFrame << ": "
                                         << ex.what());
    return;
  }
}

void EnterCart::odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
  have_received_odom_ = true;
  odom_message_ = message;
}

void EnterCart::stop() {
  twist_->linear.x = 0;
  twist_->linear.y = 0;
  twist_->linear.z = 0;
  twist_->angular.x = 0;
  twist_->angular.y = 0;
  twist_->angular.z = 0;
}

void EnterCart::turn() {
  stop();
  twist_->angular.z = this->kRightAngularVelocity;
}

void EnterCart::forward() {
  stop();
  twist_->linear.x = kLinearVelocity;
}

void EnterCart::backward() {
  stop();
  twist_->linear.x = -kLinearVelocity;
}

void EnterCart::publish_twist() {
  log_velocity();
  cmd_vel_publisher_->publish(*twist_);
}

void EnterCart::log_state_verbose() const {
  RCLCPP_DEBUG(get_logger(), state_string(state_));
}

void EnterCart::log_state() const {
  RCLCPP_INFO(get_logger(), state_string(state_));
}

void EnterCart::log_velocity() const {
  RCLCPP_DEBUG_STREAM(get_logger(), "linear twist: ("
                                        << twist_->linear.x << ", "
                                        << twist_->linear.y << ", "
                                        << twist_->linear.z << ")");
  RCLCPP_DEBUG_STREAM(get_logger(), "angular twist: ("
                                        << twist_->angular.x << ", "
                                        << twist_->angular.y << ", "
                                        << twist_->angular.z << ")");
}

std::string EnterCart::state_string(EnterCartState state) {
  std::string string_value;
  switch (state) {
  case EnterCartState::initialize:
    string_value = "Initializing";
    break;
  case EnterCartState::align_with_cart_orientation01:
    string_value = "Aligning with the cart's orientation 1";
    break;
  case EnterCartState::align_perpindicular_to_cart_orientation:
    string_value = "Aligning perpindicular to the cart's orientation";
    break;
  case EnterCartState::move_in_front_of_cart:
    string_value = "Moving in front of the cart";
    break;
  case EnterCartState::align_with_cart_orientation02:
    string_value = "Aligning with the cart's orientation 2";
    break;
  case EnterCartState::move_into_cart:
    string_value = "Moving into the cart";
    break;
  case EnterCartState::move_to_cart_center:
    string_value = "Moving into the center of the cart";
    break;
  case EnterCartState::ready_to_attach:
    string_value = "Ready to attach to the cart";
    break;
  default:
    string_value = "Error: unknown robot state";
    break;
  }
  return string_value;
}

const std::string EnterCart::kScanTopic = "scan";
const std::string EnterCart::kCmdVelTopic = "robot/cmd_vel";
const std::string EnterCart::kParentTfFrame = "robot_front_laser_link";
const std::string EnterCart::kChildTfFrame = "static_cart";
const std::string EnterCart::kOdomTopic = "odom";
const float EnterCart::kPiOverTwo = M_PI / 2.0f;

} // namespace project_part2

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<project_part2::EnterCart> pre_approach =
      std::make_shared<project_part2::EnterCart>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pre_approach);
  executor.spin();

  // Shutdown and exit.
  rclcpp::shutdown();
  return 0;
}