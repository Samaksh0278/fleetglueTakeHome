#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fleetglue_cpp/action/message.hpp"

// Define Node2 class to implement an Action Server in ROS 2
class Node2 : public rclcpp::Node {
public:
  using MessageAction = fleetglue_cpp::action::Message;                  // Define the action type
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<MessageAction>; // Define the goal handle type for the action server

  // Constructor for Node2
  Node2() : Node("RN2") {
    // Create an action server for "message_action" and bind goal, cancel, and acceptance callbacks
    this->action_server_ = rclcpp_action::create_server<MessageAction>(
      this,
      "message_action",
      std::bind(&Node2::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Node2::handle_cancel, this, std::placeholders::_1),
      std::bind(&Node2::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<MessageAction>::SharedPtr action_server_; // Shared pointer to the action server

  // Handle new goal requests from action clients
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MessageAction::Goal> goal) {
    // Log the received goal's mission data
    RCLCPP_INFO(this->get_logger(), "Received goal request with mission data: %s", goal->mission_data.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept the goal for execution
  }

  // Handle requests to cancel goals from action clients
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMessage> goal_handle) {
    // Log the cancellation request
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT; // Accept the cancellation request
  }

  // Handle accepted goals by spawning a new thread to execute the goal
  void handle_accepted(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    // Start a new thread to handle the execution of the goal to avoid blocking
    std::thread{std::bind(&Node2::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Execute callback: processes the goal and provides feedback and result
  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    // Log the start of goal execution
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // Send initial feedback to the client about the goal status
    auto feedback = std::make_shared<MessageAction::Feedback>();
    feedback->status = "Processing mission data";
    goal_handle->publish_feedback(feedback);  // Publish feedback to client
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->status.c_str());

    // Simulate a delay in processing (for demonstration purposes)
    rclcpp::Rate rate(1.0);
    rate.sleep();

    // Complete the goal execution with a successful result
    auto result = std::make_shared<MessageAction::Result>();
    result->success = true;                    // Indicate success in the result
    goal_handle->succeed(result);              // Mark goal as succeeded
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);                    // Initialize ROS 2
  auto node = std::make_shared<Node2>();       // Create an instance of Node2
  rclcpp::spin(node);                          // Keep the node active
  rclcpp::shutdown();                          // Shutdown ROS 2 when done
  return 0;
}
