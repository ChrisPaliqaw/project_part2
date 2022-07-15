#ifndef PROJECT_PART2_MATH_LINEAR_ALGEBRA_UTILITIES_HPP_
#define PROJECT_PART2_MATH_LINEAR_ALGEBRA_UTILITIES_HPP_

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace project_part2 {

geometry_msgs::msg::Vector3 euler_from_quaternion(tf2::Quaternion q);
double magnitude(geometry_msgs::msg::Vector3 v3);

} // namespace project_part2
#endif // PROJECT_PART2_MATH_LINEAR_ALGEBRA_UTILITIES_HPP_