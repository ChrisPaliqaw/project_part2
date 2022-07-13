#include <functional>
#include <numeric>
#include <string>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
#include "../include/project_part2/detect_shelf.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/logging.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
        1s,
        std::bind(&DetectShelf::timer_callback, this),
        callback_group_);

    rclcpp::SubscriptionOptions laser_options;
    laser_options.callback_group = callback_group_;

    laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        kScanTopic,
        10,
        std::bind(&DetectShelf::scanCallback, this, _1),
        laser_options);

    RCLCPP_INFO_STREAM(this->get_logger(), "Create DetectShelf");

    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
}

// Note that the z is ignored
std::pair<float, float> DetectShelf::yawAndDistanceToRosXY(const float yaw, float distance)
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
float DetectShelf::surfaceNormal(const double x1, const double y1, const double x2, const double y2)
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
    // Cast from unsigned long
    int center_plate_index = int(DetectShelf::getAverageHighIntensityIndex(laser_scan_));
    if (center_plate_index == kIndexFailureValue) {
        return;
    }
    RCLCPP_DEBUG_STREAM(
        get_logger(),
        "center_plate_index = " << center_plate_index << " / " << laser_scan_->intensities.size());
    
    std::vector<int> left_indexes, right_indexes;
    std::vector<float> left_ranges, right_ranges;
    for (unsigned long index = 0; index < laser_scan_->intensities.size(); ++index) {
        int current_intensity = int(round(laser_scan_->intensities[index]));
        if (current_intensity > 0) {
            RCLCPP_DEBUG_STREAM(get_logger(), "current_intensity = " << current_intensity);
        }
        if (current_intensity == DetectShelf::kPlateIntensity) {
            if (int(index) < center_plate_index) {
                left_indexes.push_back(int(index));
                left_ranges.push_back(laser_scan_->ranges[index]);
            } else {
                right_indexes.push_back(int(index));
                right_ranges.push_back(laser_scan_->ranges[index]);
            }
        }
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "left_ranges.size() = " << left_ranges.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "right_ranges.size() = " << right_ranges.size());

    float left_range_total = std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0);
    RCLCPP_DEBUG_STREAM(get_logger(), "left_range_total = " << left_range_total);
    float left_plate_range = left_range_total / left_ranges.size();
    RCLCPP_DEBUG_STREAM(
        get_logger(), "left_plate_range = " << left_plate_range);

    float right_range_total = std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0);
    RCLCPP_DEBUG_STREAM(get_logger(), "right_range_total = " << right_range_total);
    float right_plate_range = right_range_total / right_ranges.size();
    RCLCPP_DEBUG_STREAM(
        get_logger(), "right_plate_range = " << right_plate_range);

    int left_plate_index_total = std::accumulate(left_indexes.begin(), left_indexes.end(), 0);
    RCLCPP_DEBUG_STREAM(get_logger(), "left_plate_index_total = " << left_plate_index_total);
    int left_plate_index = left_plate_index_total / int(left_indexes.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "left_plate_index = " << left_plate_index);

    int right_plate_index_total = std::accumulate(right_indexes.begin(), right_indexes.end(), 0);
    RCLCPP_DEBUG_STREAM(get_logger(), "right_plate_index_total = " << right_plate_index_total);
    int right_plate_index = right_plate_index_total / int(right_indexes.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "right_plate_index = " << right_plate_index);

    // center index is the straight ahead of the robot - it could actually be
    // calculated only once per instantiation of the node
    auto center_index = laser_scan_->intensities.size() / 2;
    RCLCPP_DEBUG_STREAM(get_logger(), "center_index = " << center_index);
    RCLCPP_DEBUG_STREAM(get_logger(), "laser_scan_->angle_increment = " << laser_scan_->angle_increment);
    float left_plate_yaw = (float(center_index) - float(left_plate_index)) * laser_scan_->angle_increment;
    RCLCPP_DEBUG_STREAM(get_logger(), "left_plate_yaw = " << left_plate_yaw);
    float right_plate_yaw = (float(center_index) - float(right_plate_index)) * laser_scan_->angle_increment;
    RCLCPP_DEBUG_STREAM(get_logger(), "right_plate_yaw = " << right_plate_yaw);

    std::pair<float, float> left_tf = yawAndDistanceToRosXY(left_plate_yaw, left_plate_range);
    std::pair<float, float> right_tf = yawAndDistanceToRosXY(right_plate_yaw, right_plate_range);

    RCLCPP_INFO_STREAM(get_logger(), "left tf = " << left_tf.first << ", " << left_tf.second);
    RCLCPP_INFO_STREAM(get_logger(), "right tf = " << right_tf.first << ", " << right_tf.second);

    float slope_surface_normal = surfaceNormal(
        left_tf.first, left_tf.second, right_tf.first, right_tf.second);
    RCLCPP_INFO_STREAM(get_logger(), "slope_surface_normal = " << slope_surface_normal);
    float radians_surface_normal = atan(slope_surface_normal);

    tf2::Quaternion orientation_quaternion;
    orientation_quaternion.setRPY(0, 0, radians_surface_normal); 

    std::pair<double, double> center_tf;
    center_tf.first = (left_tf.first + right_tf.first) / 2;
    center_tf.second = (left_tf.second + right_tf.second) / 2;

    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = DetectShelf::kRobotLaserBaseLink;
    t.child_frame_id = DetectShelf::kCartFrame;
    t.transform.translation.x = center_tf.first;
    t.transform.translation.y = center_tf.second;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = orientation_quaternion.x();
    t.transform.rotation.y = orientation_quaternion.y();
    t.transform.rotation.z = orientation_quaternion.z();
    t.transform.rotation.w = orientation_quaternion.w();

    tf_publisher_->sendTransform(t);
}

/*
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
*/

void DetectShelf::scanCallback([[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr message)
{
    laser_scan_ = message;
}

unsigned long DetectShelf::getAverageHighIntensityIndex(sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
    std::vector<unsigned long> high_intensity_indexes;
    for (unsigned long i = 0; i < laser_scan->intensities.size(); ++i) {
        if (laser_scan->intensities[i] == DetectShelf::kPlateIntensity) {
            high_intensity_indexes.push_back(i);
        }
    }
    if (high_intensity_indexes.size() < DetectShelf::kPlateDetectionFailureThreshold) {
        RCLCPP_INFO(this->get_logger(), "Signal below threshold");
        return kIndexFailureValue;
    } else {
        long sum = std::accumulate(high_intensity_indexes.begin(), high_intensity_indexes.end(), 0);
        return sum / high_intensity_indexes.size();
    }
}

/*
std::pair<double, double> DetectShelf::tfRelativeToLaser(const double yaw, const double distance)
{
    // double adjusted_yaw = yaw - base_link_rot_.z;
    std::pair<double, double> return_value = yawAndDistanceToRosXY(yaw, distance);
    // return_value.first += base_link_trans_.x;
    // return_value.second -= base_link_trans_.y;
    return return_value;
}
*/

const std::string DetectShelf::kScanTopic = "scan";
// const std::string DetectShelf::kOdomTopic = "odom";
const std::string DetectShelf::kCartFrame = "static_cart";
const std::string DetectShelf::kRobotLaserBaseLink = "robot_front_laser_link";
// const std::string DetectShelf::kOdomFrame = "robot_odom";

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