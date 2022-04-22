#include <rclcpp/rclcpp.hpp>
#include "project_part2/subs_scan_pub_cmd.hpp"
#include "../include/project_part2/subs_scan_pub_cmd.hpp"

namespace project_part2
{
SubsScanPubCmd::SubsScanPubCmd(
        const rclcpp::NodeOptions & options,
        std::string scan_topic,
        std::string cmd_vel_topic):
    Node("subs_scan_pub_cmd"), scan_topic(scan_topic), cmd_vel_topic(cmd_vel_topic)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
}
} // namespace project_part2

/*
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubsScanPubCmd>();
  
  rclcpp::spin(node);
  return 0;
}
*/

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(project_part2::SubsScanPubCmd)