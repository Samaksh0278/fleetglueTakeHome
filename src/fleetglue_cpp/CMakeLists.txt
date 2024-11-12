cmake_minimum_required(VERSION 3.8)
project(fleetglue_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the custom action file
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Message.action"
  DEPENDENCIES action_msgs
)

# Define and link node1
add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1 rclcpp rclcpp_action)
target_link_libraries(node1 CURL::libcurl jsoncpp_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Define and link node2
add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2 rclcpp rclcpp_action)
target_link_libraries(node2 ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install executables
install(TARGETS
  node1
  node2
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
