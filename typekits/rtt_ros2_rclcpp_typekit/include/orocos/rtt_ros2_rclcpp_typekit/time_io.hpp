// Copyright 2020 Intermodalics BVBA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_IO_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_IO_HPP_

#include <iostream>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rmw/types.h"

#include "time_conversions.hpp"
#include "wrapped_duration.hpp"

// Stream operators for WrappedDuration - in same namespace for ADL
namespace rtt_ros2_rclcpp_typekit
{

inline std::ostream & operator<<(std::ostream & os, const WrappedDuration & t)
{
  return os << t.seconds();
}

inline std::istream & operator>>(std::istream & is, WrappedDuration & t)
{
  double d = 0.0;
  if (is >> d) {
    t = double_to_duration(d);
  }
  return is;
}

}  // namespace rtt_ros2_rclcpp_typekit

// Stream operators for rclcpp::Time - in rclcpp namespace for ADL
namespace rclcpp
{

inline std::ostream & operator<<(std::ostream & os, const rclcpp::Time & t)
{
  return os << t.seconds();
}

inline std::istream & operator>>(std::istream & is, rclcpp::Time & t)
{
  double d = 0.0;
  if (is >> d) {
    t = rtt_ros2_rclcpp_typekit::double_to_time(d);
  }
  return is;
}

// Stream operators for rclcpp::Duration - in rclcpp namespace for ADL
inline std::ostream & operator<<(std::ostream & os, const rclcpp::Duration & t)
{
  return os << t.seconds();
}

inline std::istream & operator>>(std::istream & is, rclcpp::Duration & t)
{
  double d = 0.0;
  if (is >> d) {
    t = rtt_ros2_rclcpp_typekit::double_to_duration(d);
  }
  return is;
}

}  // namespace rclcpp

// Stream operators for rmw_time_t - at global scope since it's a C struct
inline std::ostream & operator<<(std::ostream & os, const rmw_time_t & t)
{
  return os << rtt_ros2_rclcpp_typekit::rmw_time_t_to_double(t);
}

inline std::istream & operator>>(std::istream & is, rmw_time_t & t)
{
  double d = 0.0;
  if (is >> d) {
    t = rtt_ros2_rclcpp_typekit::double_to_rmw_time_t(d);
  }
  return is;
}

// Import streaming operators in namespace RTT for RTT::types::TypeStreamSelector
namespace RTT
{
using rtt_ros2_rclcpp_typekit::operator<<;
using rtt_ros2_rclcpp_typekit::operator>>;
using rclcpp::operator<<;
using rclcpp::operator>>;
using ::operator<<;
using ::operator>>;
}  // namespace RTT

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_IO_HPP_
