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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_DURATION_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_DURATION_HPP_

#include <chrono>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/duration.hpp"

namespace rtt_ros2_rclcpp_typekit
{

// Derived type for RTT, which is default constructible.
class WrappedDuration : public rclcpp::Duration
{
public:
  WrappedDuration() : rclcpp::Duration(0, 0) {}
  WrappedDuration(const rclcpp::Duration & duration)  // NOLINT(runtime/explicit)
  : rclcpp::Duration(duration) {}
  WrappedDuration(rclcpp::Duration && duration) noexcept  // NOLINT(runtime/explicit)
  : rclcpp::Duration(std::move(duration)) {}
  // Constructor from seconds and nanoseconds
  WrappedDuration(int32_t seconds, uint32_t nanoseconds)  // NOLINT(runtime/explicit)
  : rclcpp::Duration(seconds, nanoseconds) {}
  // Constructor from nanoseconds
  WrappedDuration(int64_t nanoseconds)  // NOLINT(runtime/explicit)
  : rclcpp::Duration(std::chrono::nanoseconds(nanoseconds)) {}
  // Constructor from builtin_interfaces message
  WrappedDuration(const builtin_interfaces::msg::Duration & duration_msg)  // NOLINT(runtime/explicit)
  : rclcpp::Duration(duration_msg) {}
  WrappedDuration & operator=(const rclcpp::Duration & duration)
  {
    static_cast<rclcpp::Duration &>(*this) = duration;
    return *this;
  }
  WrappedDuration & operator=(rclcpp::Duration && duration) noexcept
  {
    static_cast<rclcpp::Duration &>(*this) = std::move(duration);
    return *this;
  }
};

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_DURATION_HPP_
