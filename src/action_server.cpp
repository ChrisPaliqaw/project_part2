#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "phase2_custom_interfaces/action/go_to_loading_with_f.hpp"
#include "geometry_msgs/msg/twist.hpp"


class ActionServer : public rclcpp::Node
{
public:
  using Loading = phase2_custom_interfaces::action::GoToLoadingWithF;
  using GoalHandleLoading = rclcpp_action::ServerGoalHandle<Loading>;

  explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Loading>(
      this,
      "actionserver_as",
      std::bind(&ActionServer::handle_goal, this, _1, _2),
      std::bind(&ActionServer::handle_cancel, this, _1),
      std::bind(&ActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:
  rclcpp_action::Server<Loading>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Loading::Goal> goal [[maybe_unused]]) // Loading goal always empty
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLoading> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleLoading> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleLoading> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Loading::Feedback>();
    auto & message = feedback->status;
    message = "Starting movement...";
    auto result = std::make_shared<Loading::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < 5) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->complete = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Move robot forward and send feedback
      message = "Moving forward...";
      move.linear.x = 0.3;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->complete = true;
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class ActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}