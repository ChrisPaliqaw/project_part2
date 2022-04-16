#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include "../include/project_part2/subs_scan_pub_cmd.hpp"
#include "rclcpp/executors.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subs_scan_pub_cmd");
  // SubsScanPubCmd subs_scan_pub_cmd(node, "scan", "cmd_vel");
  
  rclcpp::spin(node);
  return 0;
}