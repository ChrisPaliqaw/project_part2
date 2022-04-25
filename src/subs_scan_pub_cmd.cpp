#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "project_part2/subs_scan_pub_cmd.hpp"
#include "../include/project_part2/subs_scan_pub_cmd.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

using std::placeholders::_1;

namespace project_part2
{
SubsScanPubCmd::SubsScanPubCmd(
        const rclcpp::NodeOptions & options,
        std::string scan_topic,
        std::string cmd_vel_topic):
    Node("subs_scan_pub_cmd"),
    scan_topic(scan_topic),
    cmd_vel_topic(cmd_vel_topic),
    state_(State::forward_01)
{
    laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10, std::bind(&SubsScanPubCmd::scan_callback, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Create SubsScanPubCmd");
    log_state();
}

void SubsScanPubCmd::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "Got the message");
    RCLCPP_DEBUG_STREAM(get_logger(), message->ranges[kFrontScanRange]);

    switch (state_)
    {
        case State::forward_01:
            if (message->ranges[kFrontScanRange] < kCloseWallDistance)
            {
                state_ = State::set_turn;
                log_state();
            }
            break;
        default:
            break;
    } 
}

void SubsScanPubCmd::log_state() const
{
    RCLCPP_INFO(get_logger(), state_string(state_));
}

const std::string SubsScanPubCmd::state_string(State state)
{
    std::string return_value;
    switch (state)
    {
    /*
    if self == State.FORWARD_01:
            return "Moving toward the back wall"
        elif self == State.SET_TURN or self == State.TURN:
            return "Turning toward the cart"
        elif self == State.FORWARD_02:
            return "Moving toward the cart"
        elif self == State.STOP:
            return "Arrived at the cart"
        else:
            raise ValueError(f"Unknown state {self}")
    */
        case State::forward_01:
            return_value = "Moving toward the back wall";
            break;
        case State::set_turn:
        case State::turn:
            return_value = "Turning toward the cart";
            break;
        case State::forward_02:
            return_value = "Moving toward the cart";
            break;
        case State::stop:
            return_value = "Arrived at the cart";
            break;
    }
    return return_value;
}

} // namespace project_part2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(project_part2::SubsScanPubCmd)