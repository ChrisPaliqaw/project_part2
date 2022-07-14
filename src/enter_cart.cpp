#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <chrono>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "../include/project_part2/enter_cart.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/vector3.hpp"

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
        1s,
        std::bind(&EnterCart::tf_timer_callback, this),
        callback_group_);

    rclcpp::SubscriptionOptions laser_options;
    laser_options.callback_group = callback_group_;

    laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        kScanTopic,
        10,
        std::bind(&EnterCart::scan_callback, this, _1),
        laser_options);

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);

    RCLCPP_INFO_STREAM(this->get_logger(), "Create EnterCart");

    twist_ = std::make_shared<geometry_msgs::msg::Twist>();

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }

    log_state();
}

double EnterCart::magnitude(geometry_msgs::msg::Vector3 v3)
{
    return sqrt(v3.x*v3.x + v3.y*v3.y + v3.z*v3.z);
}

void EnterCart::cmd_vel_timer_callback()
{
    if (is_complete_)
    {
        return;
    }
    
    publish_twist();
}

void EnterCart::tf_timer_callback()
{
    if (is_complete_)
    {
        return;
    }
}

void EnterCart::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    if (is_complete_)
    {
        return;
    }
    scan_message_ = message;
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
        case EnterCartState::align_with_cart_orientation:
            string_value = "Aligning with the cart's orientation";
            break;
        case EnterCartState::align_with_cart_y:
            string_value = "Aligning with the cart's y value";
            break;
        case EnterCartState::move_into_cart:
            string_value = "Moving into the cart";
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
const std::string EnterCart::kCmdVelTopic = "cmd_vel";

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