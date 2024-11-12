#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fleetglue_cpp/action/message.hpp"
#include <curl/curl.h>
#include <jsoncpp/json/json.h> 

class Node1 : public rclcpp::Node {
public:
  using MessageAction = fleetglue_cpp::action::Message;
  using GoalHandleMessage = rclcpp_action::ClientGoalHandle<MessageAction>;

  Node1() : Node("node1") {
    this->client_ = rclcpp_action::create_client<MessageAction>(this, "message_action");

    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Node1::check_and_send_goal, this));
  }

private:
  rclcpp_action::Client<MessageAction>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
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

    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
      RCLCPP_ERROR(this->get_logger(), "CURL request failed: %s", curl_easy_strerror(res));
      return false;
    }

    return !mission_data_.empty();  // Return true if data was received
  }

  // CURL write callback to store the response
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

    if (obj.isMember("mission_data")) {
      mission_data_ = obj["mission_data"].asString();
      RCLCPP_INFO(this->get_logger(), "Fetched mission data: %s", mission_data_.c_str());
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "Failed to parse mission data");
    return false;
  }

  // Timer callback to fetch data and send as goal
  void check_and_send_goal() {
    if (!fetch_mission_data_from_api()) {
      RCLCPP_WARN(this->get_logger(), "No mission data received from API");
      return;
    }

    // Attempt to parse only if mission_data_ is not empty
    if (!mission_data_.empty() && !parse_mission_data()) {
      // Only log if parsing fails after data is actually received
      RCLCPP_WARN(this->get_logger(), "Failed to parse mission data");
      return;
    }

    // Check if the new mission data is different from the last processed data
    if (mission_data_ == last_processed_data_) {
      RCLCPP_INFO(this->get_logger(), "No new mission data to process");
      return;  // Skip sending if data is not new
    }

    // Wait for the action server to be available
    if (!this->client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = MessageAction::Goal();
    goal_msg.mission_data = mission_data_;  // Use the fetched data

    auto send_goal_options = rclcpp_action::Client<MessageAction>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandleMessage::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded, clearing mission data");
        last_processed_data_ = mission_data_;  // Update last processed data
      } else {
        RCLCPP_WARN(this->get_logger(), "Goal failed or was canceled");
      }
    };

    this->client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node1>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
