#include <functional>
#include <string>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "../include/project_part2/detect_shelf.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

/*
ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
user:~$
*/

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
$ ros2 interface show nav_msgs/msg/Odometry
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
*/

/*
$ ros2 interface show geometry_msgs/msg/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
*/

/*
user:~/ros2_ws$ ros2 interface show geometry_msgs/msg/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
*/

/*
user:~/ros2_ws$ ros2 interface show geometry_msgs/msg/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
*/

/*
user:~$ ros2 interface show geometry_msgs/msg/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="https://index.ros.org/p/tf2/">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communicate the full reference for the transform
# in a self contained message.

# The frame id in the header is used as the reference frame of this transform.
std_msgs/Header header

# The frame id of the child frame to which this transform points.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
Transform transform
*/

/*
user:~$ ros2 interface show geometry_msgs/msg/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation
*/


namespace project_part2
{
DetectShelf::DetectShelf():


    Node("detect_shelf")
{

    tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ptr_ = this->create_wall_timer(
        0.5s,
        std::bind(&DetectShelf::timer_callback, this),
        callback_group_);

    rclcpp::SubscriptionOptions laser_options;
    laser_options.callback_group = callback_group_;
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = callback_group_;

    laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        kScanTopic,
        10,
        std::bind(&DetectShelf::scanCallback, this, _1),
        laser_options);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdomTopic,
        10,
        std::bind(&DetectShelf::odomCallback, this, _1),
        odom_options);

    RCLCPP_INFO_STREAM(this->get_logger(), "Create DetectShelf");

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
}

// Note that the z is ignored
std::pair<double, double> DetectShelf::yawAndDistanceToRosXY(const double yaw, double distance)
{
    if (distance > DetectShelf::kRangeMax) {
        distance = DetectShelf::kRangeMax;
    }
    // RCLCPP_INFO_STREAM(get_logger(), "New distance is {distance}");
    double theta = (DetectShelf::kPi / 2.0) + yaw;
    double y = -(distance * cos(theta));
    double x = distance * sin(theta);
    std::pair<double, double> return_value(x, y);
    return return_value;
}

// https://stackoverflow.com/questions/1243614/how-do-i-calculate-the-normal-vector-of-a-line-segment#:~:text=The%20normal%20vector%20(x'%2C,or%20(dx%2Cdy)%20.&text=Show%20activity%20on%20this%20post.,-m1%20%3D%20(y2%20%2D&text=m2%20%3D%20%2D1%20%2F%20m1%20%2F%2F,offset%20of%20new%20perpendicular%20line..
double DetectShelf::surfaceNormal(const double x1, const double y1, const double x2, const double y2)
{
        return (x2 - x1) / (y2 - y1);
}


// Code adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
geometry_msgs::msg::Vector3 DetectShelf::eulerFromQuaternion(const tf2::Quaternion q)
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

double DetectShelf::magnitude(geometry_msgs::msg::Vector3 v3)
{
    return sqrt(v3.x*v3.x + v3.y*v3.y + v3.z*v3.z);
}

void DetectShelf::timer_callback()
{

}

void DetectShelf::odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
{
    this->base_link_trans_.x = message->pose.pose.position.x;
    this->base_link_trans_.y = message->pose.pose.position.y;
    this->base_link_trans_.z = message->pose.pose.position.z;

    tf2::Quaternion orientation_quaternion;
    tf2::convert(message->pose.pose.orientation, orientation_quaternion);
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
    auto orientation_euler = eulerFromQuaternion(orientation_quaternion);
    this->base_link_rot_.x = orientation_euler.x;
    this->base_link_rot_.y = orientation_euler.y;
    this->base_link_rot_.z = orientation_euler.z;

    is_base_link_trans_and_rot_ = true;
}

void DetectShelf::scanCallback([[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    if (!is_base_link_trans_and_rot_) {
        return;
    }

}

/*
@staticmethod
    def getAverageHighIntensityIndex(laser_scan: LaserScan) -> int:
        high_intensity_indexes = []
        for i in range(len(laser_scan.intensities)):
            if laser_scan.intensities[i] == DetectCart.PLATE_INTENSITY:
                high_intensity_indexes.append(i)
        if len(high_intensity_indexes) < DetectCart.PLATE_DETECTION_FAILURE_THRESHOLD:
            raise ValueError(f"{len(high_intensity_indexes)=} < {DetectCart.PLATE_DETECTION_FAILURE_THRESHOLD=}")
        return statistics.mean(high_intensity_indexes)*/
int DetectShelf::getAverageHighIntensityIndex([[maybe_unused]] sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
    std::vector<int> high_intensity_indexes;

    return 0;
}

std::pair<double, double> DetectShelf::tfRelativeToRobot(const double yaw, const double distance)
{
    double adjusted_yaw = yaw - base_link_rot_.z;
    std::pair<double, double> return_value = yawAndDistanceToRosXY(adjusted_yaw, distance);
    return_value.first += base_link_trans_.x;
    return_value.second -= base_link_trans_.y;
    return return_value;
}

const std::string DetectShelf::kScanTopic = "scan";
const std::string DetectShelf::kOdomTopic = "odom";
const std::string DetectShelf::kCartFrame = "static_cart";
const std::string DetectShelf::kRobotLaserBaseLink = "robot_front_laser_link";
const std::string DetectShelf::kOdomFrame = "robot_odom";

} // namespace project_part2

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  std::shared_ptr<project_part2::DetectShelf> detect_cart =
      std::make_shared<project_part2::DetectShelf>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(detect_cart);
  executor.spin();

  // Shutdown and exit.
  rclcpp::shutdown();
  return 0;
}