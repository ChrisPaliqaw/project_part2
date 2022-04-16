#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

class SubsScanPubCmd {
public:
    SubsScanPubCmd(std::shared_ptr<rclcpp::Node> node, std::string scan_topic, std::string cmd_vel_topic);
private:
    std::shared_ptr<rclcpp::Node> node;
    std::string scan_topic;
    std::string cmd_vel_topic;
};