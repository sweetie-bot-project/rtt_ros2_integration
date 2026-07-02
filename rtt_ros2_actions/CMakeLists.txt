cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_actions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_node REQUIRED)
find_package(OROCOS-RTT REQUIRED)

# The package only provides class templates: build an INTERFACE library
# carrying the include directory and the dependencies of the headers.
add_library(rtt_ros2_actions INTERFACE)
target_include_directories(rtt_ros2_actions INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/orocos>
  $<INSTALL_INTERFACE:include/orocos>
)
target_link_libraries(rtt_ros2_actions INTERFACE
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rtt_ros2_node::rtt_ros2_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rtt_ros2_actions
  EXPORT ${PROJECT_NAME}
)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/orocos)
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rtt_ros2)
ament_export_dependencies(rtt_ros2_node)

ament_package()
