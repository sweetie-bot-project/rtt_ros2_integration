cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_actions_tests)

# Skip building this package if BUILD_TESTING is OFF
option(BUILD_TESTING "Build the testing tree." ON)
if(NOT BUILD_TESTING)
  install(CODE
    "message(STATUS \"Skipping installation of package ${PROJECT_NAME} because BUILD_TESTING is OFF.\")"
  )
  return()
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_actions REQUIRED)
find_package(rtt_ros2_node REQUIRED)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# unit tests
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_action_server test/test_action_server.cpp)
orocos_configure_executable(test_action_server)
target_link_libraries(test_action_server
  rtt_ros2::rtt_ros2
  rtt_ros2_node::rtt_ros2_node
  rtt_ros2_actions::rtt_ros2_actions
)
ament_target_dependencies(test_action_server
  example_interfaces
  rclcpp
  rclcpp_action
)

# must be called *after* the targets to check exported libraries etc.
ament_package()
