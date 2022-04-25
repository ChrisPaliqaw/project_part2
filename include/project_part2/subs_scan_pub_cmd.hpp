#ifndef COMPOSITION__PROJECT_PART2_COMPONENT_HPP_
#define COMPOSITION__PROJECT_PART2_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "project_part2/visibility_control.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace project_part2
{

class SubsScanPubCmd : public rclcpp::Node {
public:
    COMPOSITION_PUBLIC
    explicit SubsScanPubCmd(const rclcpp::NodeOptions & options, std::string scan_topic="scan", std::string cmd_vel_topic="cmd_vel");
    static constexpr int kFrontScanRange = 540;
    static constexpr float kCloseWallDistance = 0.35;
    enum class State { forward_01, set_turn, turn, forward_02, stop };
private:
    const std::string scan_topic;
    const std::string cmd_vel_topic;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message);
    State state_;
    static const std::string state_string(State state);
    void log_state() const;
};
} // namespace project_part2
#endif  // COMPOSITION__PROJECT_PART2_COMPONENT_HPP_