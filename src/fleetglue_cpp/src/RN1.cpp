#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fleetglue_cpp/action/message.hpp"
#include <curl/curl.h>
#include <jsoncpp/json/json.h> 

// Define the Node1 class which inherits from rclcpp::Node
class Node1 : public rclcpp::Node {
public:
  // Define types for the action message and goal handle
  using MessageAction = fleetglue_cpp::action::Message;
  using GoalHandleMessage = rclcpp_action::ClientGoalHandle<MessageAction>;

  // Constructor for Node1
  Node1() : Node("RN1") {
    // Create an action client for the "message_action" action
    this->client_ = rclcpp_action::create_client<MessageAction>(this, "message_action");

    // Create a timer to check for new data every second
    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Node1::check_and_send_goal, this));
  }

private:
  rclcpp_action::Client<MessageAction>::SharedPtr client_;  // Action client
  rclcpp::TimerBase::SharedPtr timer_;                      // Timer for periodic checks
  std::string mission_data_;          // Stores the latest fetched mission data
  std::string last_processed_data_;    // Stores the last successfully processed mission data

  // Function to perform a GET request and store the response in `mission_data_`
  bool fetch_mission_data_from_api() {
    CURL *curl;
    CURLcode res;
    std::string api_url = "http://localhost:3000/api/mission";  // API endpoint

    curl = curl_easy_init();
    if (!curl) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
      return false;
    }

    mission_data_.clear();  // Clear previous data before fetching new data

    curl_easy_setopt(curl, CURLOPT_URL, api_url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &mission_data_);

    // Perform the GET request and store the result in mission_data_
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    // Check if the request was successful
    if (res != CURLE_OK) {
      RCLCPP_ERROR(this->get_logger(), "CURL request failed: %s", curl_easy_strerror(res));
      return false;
    }

    return !mission_data_.empty();  // Return true if data was received
  }

  // CURL write callback to store the response data
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
  }

  // Function to parse JSON and extract mission data
  bool parse_mission_data() {
    Json::Reader reader;
    Json::Value obj;
    if (!reader.parse(mission_data_, obj)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON");
      return false;
    }

    // Check for "mission_data" in JSON and log the data if found
    if (obj.isMember("mission_data")) {
      mission_data_ = obj["mission_data"].asString();
      RCLCPP_INFO(this->get_logger(), "Fetched mission data: %s", mission_data_.c_str());
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "Failed to parse mission data");
    return false;
  }

  // Timer callback function to fetch data and send it as a goal
  void check_and_send_goal() {
    // Fetch data from the API and log a warning if no data is received
    if (!fetch_mission_data_from_api()) {
      RCLCPP_WARN(this->get_logger(), "No mission data received from API");
      return;
    }

    // Parse the fetched data only if it is not empty
    if (!mission_data_.empty() && !parse_mission_data()) {
      // Log if JSON parsing fails
      RCLCPP_WARN(this->get_logger(), "Failed to parse mission data");
      return;
    }

    // Check if the new data is different from the last processed data
    if (mission_data_ == last_processed_data_) {
      RCLCPP_INFO(this->get_logger(), "No new mission data to process");
      return;  // Skip sending if data is not new
    }

    // Wait for the action server to be available, timeout if not available
    if (!this->client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(), "Action server not available");
      return;
    }

    // Create a goal message with the latest mission data
    auto goal_msg = MessageAction::Goal();
    goal_msg.mission_data = mission_data_;  // Use the fetched data

    // Define options for sending the goal and handle the result callback
    auto send_goal_options = rclcpp_action::Client<MessageAction>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandleMessage::WrappedResult & result) {
      // Check if the goal succeeded and update last processed data if so
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded, clearing mission data");
        last_processed_data_ = mission_data_;  // Update last processed data
      } else {
        RCLCPP_WARN(this->get_logger(), "Goal failed or was canceled");
      }
    };

    // Send the goal asynchronously with the defined options
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);  // Initialize the ROS 2 communication
  auto node = std::make_shared<Node1>();  // Create an instance of Node1
  rclcpp::spin(node);  // Keep the node running
  rclcpp::shutdown();  // Shutdown ROS 2 communication
  return 0;
}
