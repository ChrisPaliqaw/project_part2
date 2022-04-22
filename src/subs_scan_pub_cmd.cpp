#include <rclcpp/rclcpp.hpp>
#include "../include/project_part2/subs_scan_pub_cmd.hpp"

SubsScanPubCmd::SubsScanPubCmd( 
        std::string scan_topic,
        std::string cmd_vel_topic):
    Node("subs_scan_pub_cmd_node"), scan_topic(scan_topic), cmd_vel_topic(cmd_vel_topic)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubsScanPubCmd>();
  
  rclcpp::spin(node);
  return 0;
}