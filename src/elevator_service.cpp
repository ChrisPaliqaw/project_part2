#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "project_part2/srv/elevator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("elevator_service");
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher =
    node->create_publisher<std_msgs::msg::Empty>("elevator_up", 10);
  std::shared_ptr<std_msgs::msg::Empty> message = std::make_shared<std_msgs::msg::Empty>();
  rclcpp::WallRate loop_rate(2);

  const int attempt_limit = 2;
  int attempts = 0;
  while (rclcpp::ok() && (attempts < attempt_limit)) {
    publisher->publish(*message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
    attempts += 1;
  }
  
  rclcpp::shutdown();
  return 0;
}