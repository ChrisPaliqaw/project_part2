#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <chrono>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "project_part2/subs_scan_pub_cmd.hpp"
#include "../include/project_part2/subs_scan_pub_cmd.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/empty.hpp"

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
SubsScanPubCmd::SubsScanPubCmd(
        std::string scan_topic,
        std::string odom_topic,
        std::string cmd_vel_topic,
        std::string elevator_up_topic,
        std::string elevator_down_topic):


    Node("subs_scan_pub_cmd"),
    scan_topic(scan_topic),
    cmd_vel_topic(cmd_vel_topic),
    elevator_up_topic(elevator_up_topic),
    elevator_down_topic(elevator_down_topic),
    state_(State::forward_01)
    // state_(State::stop) // For debugging elevator
{
    this->declare_parameter<bool>(kIsGazeboParameter, false);
    this->get_parameter(kIsGazeboParameter, is_gazebo_);
    RCLCPP_INFO_STREAM(get_logger(), "is_gazebo = " << (is_gazebo_ ? "true" : "false"));
    linear_velocity_ = (is_gazebo_ ? kGazeboLinearVelocity : kLinearVelocity);

    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    timer_ptr_ = this->create_wall_timer(
        1s,
        std::bind(&SubsScanPubCmd::timer_callback, this),
        callback_group_);

    rclcpp::SubscriptionOptions laser_options;
    laser_options.callback_group = callback_group_;
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = callback_group_;

    laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic,
        10,
        std::bind(&SubsScanPubCmd::scan_callback, this, _1),
        laser_options);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        10,
        std::bind(&SubsScanPubCmd::odom_callback, this, _1),
        odom_options);

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    elevator_up_publisher_ = create_publisher<std_msgs::msg::Empty>(elevator_up_topic, 10);
    elevator_down_publisher_ = create_publisher<std_msgs::msg::Empty>(elevator_down_topic, 10);


    RCLCPP_INFO_STREAM(this->get_logger(), "Create SubsScanPubCmd");

    twist_ = std::make_shared<geometry_msgs::msg::Twist>();
    empty_ = std::make_shared<std_msgs::msg::Empty>();

    // For debugging why elevator not working
    //elevator_up_publisher_->publish(*empty_);

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }

    log_state();
}

// Code adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
geometry_msgs::msg::Vector3 SubsScanPubCmd::euler_from_quaternion(tf2::Quaternion q)
{
    geometry_msgs::msg::Vector3 v3;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double roll;
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw;
    yaw = std::atan2(siny_cosp, cosy_cosp);

    v3.x = roll;
    v3.y = pitch;
    v3.z = yaw;

    return v3;
}

double SubsScanPubCmd::magnitude(geometry_msgs::msg::Vector3 v3)
{
    return sqrt(v3.x*v3.x + v3.y*v3.y + v3.z*v3.z);
}

bool SubsScanPubCmd::is_buffer_stop_state () const
{
    switch (state_)
    {
    case State::stop_forward_01:
        [[fallthrough]];
    case State::stop_turn:
        [[fallthrough]];
    case State::stop_forward_02:
        return true;
    default:
        return false;
    }
}

bool SubsScanPubCmd::is_odom_state () const
{
    switch (state_)
    {
    case State::stop_forward_01:
        [[fallthrough]];
    case State::set_turn:
        [[fallthrough]];
    case State::turn:
        [[fallthrough]];
    case State::stop_turn:
        [[fallthrough]];
    case State::stop_forward_02:
        return true;
    default:
        return false;
    }
}

bool SubsScanPubCmd::is_scan_state () const
{
    switch (state_)
    {
    case State::stop_forward_01:
        [[fallthrough]];
    case State::set_turn:
        [[fallthrough]];
    case State::turn:
        [[fallthrough]];
    case State::stop_turn:
        [[fallthrough]];
    case State::stop_forward_02:
        return false;
    default:
        return true;
    }
}

bool SubsScanPubCmd::is_stopped(geometry_msgs::msg::Vector3 v3)
{
    return abs(magnitude(v3)) <= kTurnFuzz;
}

void SubsScanPubCmd::timer_callback()
{
    // std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_DEBUG(this->get_logger(), "Behavior timer callback");
    
    switch (state_)
    {
    case State::forward_01:
        forward();
        break;
    case State::stop_forward_01:
        stop();
        break;
    case State::set_turn:
        // Handled in odom
        break;
    case State::turn:
        turn();
        break;
    case State::stop_turn:
        stop();
        break;
    case State::forward_02:
        forward();
        break;
    case State::stop_forward_02:
        stop();
        break;
    case State::elevator_up:
        elevator_up();
        state_ = State::stop;
        log_state();
        break;
    case State::stop:
        stop();
        break;
    }
    publish_twist();
}

void SubsScanPubCmd::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
    // std::lock_guard<std::mutex> lock(state_mutex_);

    tf2::Quaternion tf2_quaternion;
    // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
    tf2::convert(message->pose.pose.orientation, tf2_quaternion);

    geometry_msgs::msg::Vector3 current_rotation_v3 = euler_from_quaternion(tf2_quaternion);

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
            case State::stop_forward_01:
                state_ = State::set_turn;
                log_state();
                break;
            case State::stop_turn:
                state_ = State::forward_02;
                log_state();
                break;
            case State::stop_forward_02:
                state_ = State::elevator_up;
                log_state();
                break;
            default:
                RCLCPP_ERROR_STREAM(get_logger(), "Unexpected state: " << state_string(state_));
            }
        }
    }
    else if (state_ == State::set_turn) {
        RCLCPP_INFO_STREAM(get_logger(), "SETTING TURN");
        tf2::Quaternion q_rot;
        // Rotate the current pose by 90 degrees about Z to get our goal pose
        q_rot.setRPY(0.0, 0.0, SubsScanPubCmd::kGoalAngularDisplacement);
        tf2::Quaternion goal_turn = q_rot * tf2_quaternion;
        goal_turn.normalize();
        goal_turn_v3_ = SubsScanPubCmd::euler_from_quaternion(goal_turn);
        state_ = State::turn;
    }
    else if ((state_ == State::turn) && (abs(current_rotation_v3.z - goal_turn_v3_.z) <= kTurnFuzz))
    {
        state_ = State::stop_turn;
        log_state();
    }
}

void SubsScanPubCmd::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    // std::lock_guard<std::mutex> lock(state_mutex_);

    if (!is_scan_state())
    {
        return;
    }
    
    RCLCPP_DEBUG_STREAM(get_logger(), message->ranges[kFrontScanRange]);
    switch (state_)
    {
        case State::forward_01:
            if (message->ranges[kFrontScanRange] < kCloseWallDistance)
            {
                state_ = State::stop_forward_01;
                log_state();
            }
            break;
        case State::forward_02:
            if (message->ranges[kFrontScanRange] < kCloseCartDistance)
            {
                state_ = State::stop_forward_02;
                log_state();
            }
            break;
        default:
            break;
    }
}

void SubsScanPubCmd::stop()
{
    twist_->linear.x = 0;
    twist_->linear.y = 0;
    twist_->linear.z = 0;
    twist_->angular.x = 0;
    twist_->angular.y = 0;
    twist_->angular.z = 0;
}

void SubsScanPubCmd::turn()
{
    stop();
    twist_->angular.z = this->kRightAngularVelocity;
}

void SubsScanPubCmd::forward()
{
    stop();
    twist_->linear.x = linear_velocity_;
}

void SubsScanPubCmd::elevator_up()
{
    RCLCPP_INFO(get_logger(), "Publishing elevator_up");
    elevator_up_publisher_->publish(*empty_);
}

void SubsScanPubCmd::publish_twist()
{
    log_velocity();
    cmd_vel_publisher_->publish(*twist_);
}

void SubsScanPubCmd::log_state_verbose() const
{
    RCLCPP_DEBUG(get_logger(), state_string(state_));
    RCLCPP_DEBUG_STREAM(get_logger(), "linear: (" << twist_->linear.x << ", "  << twist_->linear.y << ", "  << twist_->linear.z << ")");
    RCLCPP_DEBUG_STREAM(get_logger(), "angular: (" << twist_->angular.x << ", "  << twist_->angular.y << ", "  << twist_->angular.z << ")");
}

void SubsScanPubCmd::log_state() const
{
    RCLCPP_INFO(get_logger(), state_string(state_));
}

void SubsScanPubCmd::log_velocity() const
{
    RCLCPP_INFO_STREAM(get_logger(), "linear: (" << twist_->linear.x << ", "  << twist_->linear.y << ", "  << twist_->linear.z << ")");
    RCLCPP_INFO_STREAM(get_logger(), "angular: (" << twist_->angular.x << ", "  << twist_->angular.y << ", "  << twist_->angular.z << ")");
}

std::string SubsScanPubCmd::state_string(State state)
{
    std::string string_value;
    switch (state)
    {
        case State::forward_01:
            string_value = "Moving toward the back wall";
            break;
        case State::stop_forward_01:
            string_value = "Stopping after moving toward the back wall";
            break;
        case State::set_turn:
            string_value = "Setting the turn speed";
            break;
        case State::turn:
            string_value = "Turning toward the cart";
            break;
        case State::stop_turn:
            string_value = "Stopping after turning toward the cart";
            break;
        case State::forward_02:
            string_value = "Moving toward the cart";
            break;
        case State::stop_forward_02:
            string_value = "Stopping inside the cart";
            break;
        case State::elevator_up:
            string_value = "Raising the elevator";
            break;
        case State::stop:
            string_value = "Inside the cart with the elevator raised";
            break;
        default:
            string_value = "Error: unknown robot state";
            break;
    }
    return string_value;
}

const double SubsScanPubCmd::kGoalAngularDisplacement = -(M_PI / 2.0);
const std::string SubsScanPubCmd::kIsGazeboParameter = "is_gazebo";

} // namespace project_part2

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  std::shared_ptr<project_part2::SubsScanPubCmd> subs_scan_pub_cmd =
      std::make_shared<project_part2::SubsScanPubCmd>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(subs_scan_pub_cmd);
  executor.spin();

  // Shutdown and exit.
  rclcpp::shutdown();
  return 0;
}