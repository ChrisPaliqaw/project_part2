#ifndef COMPOSITION__PROJECT_PART2_COMPONENT_HPP_
#define COMPOSITION__PROJECT_PART2_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "project_part2/visibility_control.h"

namespace project_part2
{

class SubsScanPubCmd : public rclcpp::Node {
public:
    COMPOSITION_PUBLIC
    explicit SubsScanPubCmd(const rclcpp::NodeOptions & options, std::string scan_topic="scan", std::string cmd_vel_topic="cmd_vel");
private:
    std::string scan_topic;
    std::string cmd_vel_topic;
};
} // namespace project_part2
#endif  // COMPOSITION__PROJECT_PART2_COMPONENT_HPP_