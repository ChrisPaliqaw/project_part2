#include <rclcpp/rclcpp.hpp>
#include "../include/project_part2/pre_approach.hpp"

int main(int argc, char *argv[]) {
  // Some initialization.
  rclcpp::init(argc, argv);

  // Instantiate a node.
  std::shared_ptr<project_part2::PreApproach> node =
      std::make_shared<project_part2::PreApproach>();

  // Same code but in steps
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Shutdown and exit.
  rclcpp::shutdown();
  return 0;
}