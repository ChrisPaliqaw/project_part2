#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <chrono>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <linear_algebra_tools/linear_algebra.hpp>
#include "../include/project_part2/pre_approach.hpp"

using namespace std::chrono_literals;

/*
ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
user:~$
*/

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
$ ros2 interface show nav_msgs/msg/Odometry
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
*/

/*
$ ros2 interface show geometry_msgs/msg/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
*/

/*
user:~/ros2_ws$ ros2 interface show geometry_msgs/msg/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
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
PreApproach::PreApproach(
        std::string scan_topic,
        std::string odom_topic,
        std::string cmd_vel_topic,
        std::string elevator_up_topic,
        std::string elevator_down_topic):


    Node("pre_approach"),
    scan_topic(scan_topic),
    cmd_vel_topic(cmd_vel_topic),
    elevator_up_topic(elevator_up_topic),
    elevator_down_topic(elevator_down_topic),
    state_(PreApproachState::forward_01)
{
    this->declare_parameter<bool>(kIsGazeboParameter, false);
    this->get_parameter(kIsGazeboParameter, is_gazebo_);
    RCLCPP_INFO_STREAM(get_logger(), "is_gazebo = " << (is_gazebo_ ? "true" : "false"));
    linear_velocity_ = (is_gazebo_ ? kGazeboLinearVelocity : kLinearVelocity);

    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    timer_ptr_ = this->create_wall_timer(
        0.5s,
        std::bind(&PreApproach::timer_callback, this),
        callback_group_);

    rclcpp::SubscriptionOptions laser_options;
    laser_options.callback_group = callback_group_;
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = callback_group_;

    laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic,
        10,
        std::bind(&PreApproach::scan_callback, this, _1),
        laser_options);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        10,
        std::bind(&PreApproach::odom_callback, this, _1),
        odom_options);

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    RCLCPP_INFO_STREAM(this->get_logger(), "Create PreApproach");

    twist_ = std::make_shared<geometry_msgs::msg::Twist>();

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }

    log_state();
}

bool PreApproach::is_buffer_stop_state () const
{
    switch (state_)
    {
    case PreApproachState::stop_forward_01:
        [[fallthrough]];
    case PreApproachState::stop_turn:
        return true;
    default:
        return false;
    }
}

bool PreApproach::is_odom_state () const
{
    switch (state_)
    {
    case PreApproachState::stop_forward_01:
        [[fallthrough]];
    case PreApproachState::set_turn:
        [[fallthrough]];
    case PreApproachState::turn:
        [[fallthrough]];
    case PreApproachState::stop_turn:
        return true;
    default:
        return false;
    }
}

bool PreApproach::is_scan_state () const
{
    switch (state_)
    {
    case PreApproachState::stop_forward_01:
        [[fallthrough]];
    case PreApproachState::set_turn:
        [[fallthrough]];
    case PreApproachState::turn:
        [[fallthrough]];
    case PreApproachState::stop_turn:
        return false;
    default:
        return true;
    }
}

bool PreApproach::is_stopped(geometry_msgs::msg::Vector3 v3)
{
    return abs(linear_algebra_utilities::Magnitude(v3)) <= kTurnFuzz;
}

void PreApproach::timer_callback()
{
    if (is_complete_)
    {
        return;
    }
    // std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_DEBUG(this->get_logger(), "Behavior timer callback");
    
    switch (state_)
    {
    case PreApproachState::forward_01:
        forward();
        break;
    case PreApproachState::stop_forward_01:
        stop();
        break;
    case PreApproachState::set_turn:
        // Handled in odom
        break;
    case PreApproachState::turn:
        turn();
        break;
    case PreApproachState::stop_turn:
        stop();
        break;
    case PreApproachState::stop:
        stop();
        is_complete_ = true;
        break;
    }
    publish_twist();
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
    if (is_complete_)
    {
        return;
    }
    // std::lock_guard<std::mutex> lock(state_mutex_);

    tf2::Quaternion tf2_quaternion;
    // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
    tf2::convert(message->pose.pose.orientation, tf2_quaternion);

    geometry_msgs::msg::Vector3 current_rotation_v3 = linear_algebra_utilities::EulerFromQuaternion(tf2_quaternion);

    // Only turning behavior is determined by odom
    if (!is_odom_state())
    {
        return;
    }
    else if (is_buffer_stop_state())
    {
        bool stopped = is_stopped(message->twist.twist.angular) && is_stopped(message->twist.twist.linear);
        if (stopped)
        {
            switch (state_) {
            case PreApproachState::stop_forward_01:
                state_ = PreApproachState::set_turn;
                log_state();
                break;
            case PreApproachState::stop_turn:
                state_ = PreApproachState::stop;
                log_state();
                break;
            default:
                RCLCPP_ERROR_STREAM(get_logger(), "Unexpected state: " << state_string(state_));
            }
        }
    }
    else if (state_ == PreApproachState::set_turn) {
        RCLCPP_INFO_STREAM(get_logger(), "SETTING TURN");
        tf2::Quaternion q_rot;
        // Rotate the current pose by 90 degrees about Z to get our goal pose
        q_rot.setRPY(0.0, 0.0, PreApproach::kGoalAngularDisplacement);
        tf2::Quaternion goal_turn = q_rot * tf2_quaternion;
        goal_turn.normalize();
        goal_turn_v3_ = linear_algebra_utilities::EulerFromQuaternion(goal_turn);
        state_ = PreApproachState::turn;
    }
    else if ((state_ == PreApproachState::turn) && (abs(current_rotation_v3.z - goal_turn_v3_.z) <= kTurnFuzz))
    {
        state_ = PreApproachState::stop_turn;
        log_state();
    }
}

void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    // std::lock_guard<std::mutex> lock(state_mutex_);
    if (is_complete_)
    {
        return;
    }
    if (!is_scan_state())
    {
        return;
    }
    
    RCLCPP_DEBUG_STREAM(get_logger(), message->ranges[kFrontScanRange]);
    switch (state_)
    {
        case PreApproachState::forward_01:
            if (message->ranges[kFrontScanRange] < kCloseWallDistance)
            {
                state_ = PreApproachState::stop_forward_01;
                log_state();
            }
            break;
        default:
            break;
    }
}

void PreApproach::stop()
{
    twist_->linear.x = 0;
    twist_->linear.y = 0;
    twist_->linear.z = 0;
    twist_->angular.x = 0;
    twist_->angular.y = 0;
    twist_->angular.z = 0;
}

void PreApproach::turn()
{
    stop();
    twist_->angular.z = this->kRightAngularVelocity;
}

void PreApproach::forward()
{
    stop();
    twist_->linear.x = linear_velocity_;
}

void PreApproach::publish_twist()
{
    log_velocity();
    cmd_vel_publisher_->publish(*twist_);
}

void PreApproach::log_state_verbose() const
{
    RCLCPP_DEBUG(get_logger(), state_string(state_));
    RCLCPP_DEBUG_STREAM(get_logger(), "linear: (" << twist_->linear.x << ", "  << twist_->linear.y << ", "  << twist_->linear.z << ")");
    RCLCPP_DEBUG_STREAM(get_logger(), "angular: (" << twist_->angular.x << ", "  << twist_->angular.y << ", "  << twist_->angular.z << ")");
}

void PreApproach::log_state() const
{
    RCLCPP_INFO(get_logger(), state_string(state_));
}

void PreApproach::log_velocity() const
{
    RCLCPP_INFO_STREAM(get_logger(), "linear: (" << twist_->linear.x << ", "  << twist_->linear.y << ", "  << twist_->linear.z << ")");
    RCLCPP_INFO_STREAM(get_logger(), "angular: (" << twist_->angular.x << ", "  << twist_->angular.y << ", "  << twist_->angular.z << ")");
}

std::string PreApproach::state_string(PreApproachState state)
{
    std::string string_value;
    switch (state)
    {
        case PreApproachState::forward_01:
            string_value = "Moving toward the back wall";
            break;
        case PreApproachState::stop_forward_01:
            string_value = "Stopping after moving toward the back wall";
            break;
        case PreApproachState::set_turn:
            string_value = "Setting the turn speed";
            break;
        case PreApproachState::turn:
            string_value = "Turning toward the cart";
            break;
        case PreApproachState::stop_turn:
            string_value = "Stopping after turning toward the cart";
            break;
        case PreApproachState::stop:
            string_value = "Facing the cart";
            break;
        default:
            string_value = "Error: unknown robot state";
            break;
    }
    return string_value;
}

const double PreApproach::kGoalAngularDisplacement = -(M_PI / 2.0);
const std::string PreApproach::kIsGazeboParameter = "is_gazebo";

} // namespace project_part2

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  std::shared_ptr<project_part2::PreApproach> pre_approach =
      std::make_shared<project_part2::PreApproach>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pre_approach);
  executor.spin();

  // Shutdown and exit.
  rclcpp::shutdown();
  return 0;
}