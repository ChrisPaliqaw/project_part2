#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <chrono>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "../include/project_part2/enter_cart.hpp"
#include "../include/project_part2/linear_algebra_utilities.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

/*
user:~$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
*/

/*
user:~$ ros2 interface show geometry_msgs/msg/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

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

namespace project_part2
{
EnterCart::EnterCart():
    Node("enter_cart"),
    state_(EnterCartState::initialize)
{
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(* tf_buffer_);

    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    cmd_vel_timer_ptr_ = this->create_wall_timer(
        0.5s,
        std::bind(&EnterCart::cmd_vel_timer_callback, this),
        callback_group_);
    tf_timer_ptr_ = this->create_wall_timer(
        0.5s,
        std::bind(&EnterCart::tf_timer_callback, this),
        callback_group_);

    
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = callback_group_;
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdomTopic,
        10,
        std::bind(&EnterCart::odomCallback, this, _1),
        odom_options);

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);

    RCLCPP_INFO_STREAM(this->get_logger(), "Create EnterCart");

    twist_ = std::make_shared<geometry_msgs::msg::Twist>();

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }

    log_state();
}

void EnterCart::cmd_vel_timer_callback()
{
    if (is_complete_)
    {
        return;
    }

    log_state_verbose();

    if (state_ == EnterCartState::initialize) {
        if (have_received_tf_) {
            state_ = EnterCartState::align_perpindicular_to_cart_orientation;
            log_state();
        }
        return;
    }

    tf2::Quaternion tf2_quat_from_msg;
    tf2::fromMsg(transform_stamped_.transform.rotation, tf2_quat_from_msg);

    geometry_msgs::msg::Vector3 cart_euler =
        euler_from_quaternion(tf2_quat_from_msg);
    RCLCPP_DEBUG_STREAM(get_logger(), "Cart rotation: (" <<
        cart_euler.x << ", " <<
        cart_euler.y << ", " <<
        cart_euler.z << ")");
    RCLCPP_DEBUG_STREAM(get_logger(), "Cart translation: (" <<
        transform_stamped_.transform.translation.x << ", " <<
        transform_stamped_.transform.translation.y << ", " <<
        transform_stamped_.transform.translation.z << ")");

    float y_translation = transform_stamped_.transform.translation.y;
    float x_translation = transform_stamped_.transform.translation.x;

    float y_odom = odom_message_->pose.pose.position.y;

    // When moving in front of the cart, are we trying to point toward the starting wall or the
    // opposing wall?
    const float multiplier =  (y_translation > 0.0) ? -1.0f : 1.0f;
    const float goal_euler_z = kPiOverTwo * multiplier;
    RCLCPP_DEBUG_STREAM(get_logger(), "goal_euler_z = " << goal_euler_z);

    switch (state_) {
    case EnterCartState::align_perpindicular_to_cart_orientation:
      if (abs(cart_euler.z - goal_euler_z) <= kTurnFuzz) {
        stop();
        state_ = EnterCartState::move_in_front_of_cart;
        log_state();
      } else {
        turn();
      }
      break;
    case EnterCartState::move_in_front_of_cart:
      if (abs(y_translation) <= kTranslateFuzz) {
        stop();
        state_ = EnterCartState::align_with_cart_orientation;
        log_state();
      } else {
        forward();
      }
      break;
    case EnterCartState::align_with_cart_orientation:
        
      if (abs(cart_euler.z) <= kTurnFuzz) {
        stop();
        state_ = EnterCartState::move_into_cart;
        log_state();
      } else {
        turn();
      }
      break;
    case EnterCartState::move_into_cart:
      if (x_translation >= 0) {
        stop();
        state_ = EnterCartState::move_to_cart_center;
        goal_odom_y = y_odom - kCartMinDepthX;
        RCLCPP_INFO_STREAM(get_logger(), "goal_odom_y = " << goal_odom_y);
        log_state();
      } else {
        forward();
      }
      break;
    case EnterCartState::move_to_cart_center:
      RCLCPP_INFO_STREAM(get_logger(), "goal_odom_y = " << goal_odom_y);
      RCLCPP_INFO_STREAM(get_logger(), "y_odom  = " << y_odom);
      if (y_odom <= goal_odom_y) {
        stop();
        state_ = EnterCartState::ready_to_attach;
        log_state();
        is_complete_ = true;
      } else {
        forward();
      }
      break;
    default:
      RCLCPP_ERROR_STREAM(
        get_logger(), "Unexpected state in cmd_vel_timer_callback(): " << state_string(state_));
      break;
    }
    
    publish_twist();
}

void EnterCart::tf_timer_callback()
{
    if (is_complete_)
    {
        return;
    }

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        transform_stamped_ = tf_buffer_->lookupTransform(
          kChildTfFrame, kParentTfFrame,
          tf2::TimePointZero);
        have_received_tf_ = true;
    } catch (tf2::TransformException & ex) {
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Could not transform " << kChildTfFrame << " from " << kParentTfFrame << ": " << ex.what());
        return;
    }
}

void EnterCart::odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
{
    odom_message_ = message;
}

void EnterCart::stop()
{
    twist_->linear.x = 0;
    twist_->linear.y = 0;
    twist_->linear.z = 0;
    twist_->angular.x = 0;
    twist_->angular.y = 0;
    twist_->angular.z = 0;
}

void EnterCart::turn()
{
    stop();
    twist_->angular.z = this->kRightAngularVelocity;
}

void EnterCart::forward()
{
    stop();
    twist_->linear.x = kLinearVelocity;
}

void EnterCart::publish_twist()
{
    log_velocity();
    cmd_vel_publisher_->publish(*twist_);
}

void EnterCart::log_state_verbose() const
{
    RCLCPP_DEBUG(get_logger(), state_string(state_));
    RCLCPP_DEBUG_STREAM(get_logger(), "linear: (" << twist_->linear.x << ", "  << twist_->linear.y << ", "  << twist_->linear.z << ")");
    RCLCPP_DEBUG_STREAM(get_logger(), "angular: (" << twist_->angular.x << ", "  << twist_->angular.y << ", "  << twist_->angular.z << ")");
}

void EnterCart::log_state() const
{
    RCLCPP_INFO(get_logger(), state_string(state_));
}

void EnterCart::log_velocity() const
{
    RCLCPP_INFO_STREAM(get_logger(), "linear: (" << twist_->linear.x << ", "  << twist_->linear.y << ", "  << twist_->linear.z << ")");
    RCLCPP_INFO_STREAM(get_logger(), "angular: (" << twist_->angular.x << ", "  << twist_->angular.y << ", "  << twist_->angular.z << ")");
}

std::string EnterCart::state_string(EnterCartState state)
{
    std::string string_value;
    switch (state)
    {
        case EnterCartState::initialize:
            string_value = "Initializing";
            break;
        case EnterCartState::align_perpindicular_to_cart_orientation:
            string_value = "Aligning perpindicular to the cart's orientation";
            break;
        case EnterCartState::move_in_front_of_cart:
            string_value = "Moving in front of the cart";
            break;
        case EnterCartState::align_with_cart_orientation:
            string_value = "Aligning with the cart's orientation";
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

int main(int argc, char * argv[]) {
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