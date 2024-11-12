#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fleetglue_cpp/action/message.hpp"

class Node2 : public rclcpp::Node {
public:
  using MessageAction = fleetglue_cpp::action::Message;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<MessageAction>;

  Node2() : Node("node2") {
    this->action_server_ = rclcpp_action::create_server<MessageAction>(
      this,
      "message_action",
      std::bind(&Node2::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Node2::handle_cancel, this, std::placeholders::_1),
      std::bind(&Node2::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<MessageAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MessageAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with mission data: %s", goal->mission_data.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMessage> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    std::thread{std::bind(&Node2::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // Send feedback and set up a simple response
    auto feedback = std::make_shared<MessageAction::Feedback>();
    feedback->status = "Processing mission data";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());

    // Simulate some processing time
    rclcpp::Rate rate(1.0);
    rate.sleep();

    // Finish the action
    auto result = std::make_shared<MessageAction::Result>();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
