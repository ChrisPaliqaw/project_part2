#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
user:~$ ros2 interface show geometry_msgs/msg/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
*/

/*
user:~/ros2_ws$ ros2 interface show geometry_msgs/msg/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
*/


namespace project_part2
{

// Code adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
geometry_msgs::msg::Vector3 euler_from_quaternion(tf2::Quaternion q)
{
    geometry_msgs::msg::Vector3 v3;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double roll;
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw;
    yaw = std::atan2(siny_cosp, cosy_cosp);

    v3.x = roll;
    v3.y = pitch;
    v3.z = yaw;

    return v3;
}

double magnitude(geometry_msgs::msg::Vector3 v3)
{
    return sqrt(v3.x*v3.x + v3.y*v3.y + v3.z*v3.z);
}

} // namespace project_part2

int main(/*int argc, char * argv[]*/)
{
}