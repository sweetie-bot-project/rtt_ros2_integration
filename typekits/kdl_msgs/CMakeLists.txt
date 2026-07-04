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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
