#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

class SubsScanPubCmd : public rclcpp::Node {
public:
    SubsScanPubCmd(std::string scan_topic="scan", std::string cmd_vel_topic="cmd_vel");
private:
    std::string scan_topic;
    std::string cmd_vel_topic;
};