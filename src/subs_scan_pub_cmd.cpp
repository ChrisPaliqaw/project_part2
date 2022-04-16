#include <rclcpp/rclcpp.hpp>
#include "../include/project_part2/subs_scan_pub_cmd.hpp"

SubsScanPubCmd::SubsScanPubCmd(
        std::shared_ptr<rclcpp::Node> node,
        std::string scan_topic,
        std::string cmd_vel_topic):
    node(node), scan_topic(scan_topic), cmd_vel_topic(cmd_vel_topic)
{
    RCLCPP_INFO_STREAM(node->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
}