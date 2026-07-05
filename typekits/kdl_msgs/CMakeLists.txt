cmake_minimum_required(VERSION 3.5)
project(kdl_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector.msg"
  "msg/Rotation.msg"
  "msg/Frame.msg"
  "msg/Twist.msg"
  "msg/Wrench.msg"
  "msg/RotationalInertia.msg"
  "msg/RigidBodyInertia.msg"
  "msg/Jacobian.msg"
  "msg/JntArray.msg"
  "msg/VectorStamped.msg"
  "msg/RotationStamped.msg"
  "msg/FrameStamped.msg"
  "msg/TwistStamped.msg"
  "msg/WrenchStamped.msg"
  "msg/JntArrayStamped.msg"
  DEPENDENCIES std_msgs
)

# Hand-written KDL <-> kdl_msgs conversions header (needs orocos_kdl). Installed
# into the same include/<pkg>/<pkg> layout rosidl uses for generated headers, so
# it resolves as <kdl_msgs/conversions.hpp>.
find_package(orocos_kdl_vendor REQUIRED)
install(
  FILES include/kdl_msgs/conversions.hpp
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)
ament_export_dependencies(orocos_kdl_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
