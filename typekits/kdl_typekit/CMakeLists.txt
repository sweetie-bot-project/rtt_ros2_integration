cmake_minimum_required(VERSION 3.5)
project(kdl_typekit)

find_package(ament_cmake REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)

include_directories(include/orocos)

# Rotation properties use the roll/pitch/yaw representation.
add_definitions(-DROTATION_PROPERTIES_RPY)

orocos_typekit(${PROJECT_NAME}
  src/kdlTypekit.cpp
  src/kdlTypekitConstructors.cpp
  src/kdlTypekitOperators.cpp
  src/kdlTypekitChain.cpp
  src/kdlTypekitFrame.cpp
  src/kdlTypekitJacobian.cpp
  src/kdlTypekitJntArray.cpp
  src/kdlTypekitJoint.cpp
  src/kdlTypekitRotation.cpp
  src/kdlTypekitSegment.cpp
  src/kdlTypekitTwist.cpp
  src/kdlTypekitVector.cpp
  src/kdlTypekitWrench.cpp
  src/motionproperties.cpp
  src/kinfamproperties.cpp
)
target_link_libraries(${PROJECT_NAME} orocos-kdl)

# Note: the mqueue transport for KDL types (src/mqueue/kdl_mqueue.cpp) is not
# built yet — the ROS 2 toolchain does not export the RTT::mqueue::MQSendRecv
# symbols for linking. Add it back if inter-process mqueue transport of KDL
# types is required.

orocos_install_headers(
  FILES include/orocos/${PROJECT_NAME}/typekit/Types.hpp
  INSTALL include/orocos/${PROJECT_NAME}/typekit
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rtt_ros2 orocos_kdl_vendor)
ament_export_include_directories(include/orocos)

ament_package()
orocos_generate_package(
  DEPENDS orocos-kdl
)
