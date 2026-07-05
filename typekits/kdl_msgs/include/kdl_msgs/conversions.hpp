// Copyright 2026 Sweetie Bot Project
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

#ifndef KDL_MSGS__CONVERSIONS_HPP_
#define KDL_MSGS__CONVERSIONS_HPP_

// Conversions between the KDL C++ geometric types and the kdl_msgs ROS 2
// messages. In ROS 1 the rtt_kdl_msgs typekit represented these message fields
// directly as KDL C++ types; under rosidl the message fields are plain structs,
// so code that does KDL math on them converts explicitly with these helpers.

#include <kdl/frames.hpp>

#include "kdl_msgs/msg/vector.hpp"
#include "kdl_msgs/msg/rotation.hpp"
#include "kdl_msgs/msg/frame.hpp"
#include "kdl_msgs/msg/twist.hpp"
#include "kdl_msgs/msg/wrench.hpp"

namespace kdl_msgs
{

// --- Vector ---
inline KDL::Vector fromMsg(const kdl_msgs::msg::Vector & m)
{
  return KDL::Vector(m.x, m.y, m.z);
}
inline void toMsg(const KDL::Vector & v, kdl_msgs::msg::Vector & m)
{
  m.x = v.x();
  m.y = v.y();
  m.z = v.z();
}
inline kdl_msgs::msg::Vector toMsg(const KDL::Vector & v)
{
  kdl_msgs::msg::Vector m;
  toMsg(v, m);
  return m;
}

// --- Rotation ---
inline KDL::Rotation fromMsg(const kdl_msgs::msg::Rotation & m)
{
  KDL::Rotation r;
  for (int i = 0; i < 9; ++i) {r.data[i] = m.data[i];}
  return r;
}
inline void toMsg(const KDL::Rotation & r, kdl_msgs::msg::Rotation & m)
{
  for (int i = 0; i < 9; ++i) {m.data[i] = r.data[i];}
}
inline kdl_msgs::msg::Rotation toMsg(const KDL::Rotation & r)
{
  kdl_msgs::msg::Rotation m;
  toMsg(r, m);
  return m;
}

// --- Frame ---
inline KDL::Frame fromMsg(const kdl_msgs::msg::Frame & m)
{
  return KDL::Frame(fromMsg(m.m), fromMsg(m.p));
}
inline void toMsg(const KDL::Frame & f, kdl_msgs::msg::Frame & m)
{
  toMsg(f.p, m.p);
  toMsg(f.M, m.m);
}
inline kdl_msgs::msg::Frame toMsg(const KDL::Frame & f)
{
  kdl_msgs::msg::Frame m;
  toMsg(f, m);
  return m;
}

// --- Twist (KDL vel/rot <-> msg linear/angular) ---
inline KDL::Twist fromMsg(const kdl_msgs::msg::Twist & m)
{
  return KDL::Twist(fromMsg(m.linear), fromMsg(m.angular));
}
inline void toMsg(const KDL::Twist & t, kdl_msgs::msg::Twist & m)
{
  toMsg(t.vel, m.linear);
  toMsg(t.rot, m.angular);
}
inline kdl_msgs::msg::Twist toMsg(const KDL::Twist & t)
{
  kdl_msgs::msg::Twist m;
  toMsg(t, m);
  return m;
}

// --- Wrench ---
inline KDL::Wrench fromMsg(const kdl_msgs::msg::Wrench & m)
{
  return KDL::Wrench(fromMsg(m.force), fromMsg(m.torque));
}
inline void toMsg(const KDL::Wrench & w, kdl_msgs::msg::Wrench & m)
{
  toMsg(w.force, m.force);
  toMsg(w.torque, m.torque);
}
inline kdl_msgs::msg::Wrench toMsg(const KDL::Wrench & w)
{
  kdl_msgs::msg::Wrench m;
  toMsg(w, m);
  return m;
}

}  // namespace kdl_msgs

#endif  // KDL_MSGS__CONVERSIONS_HPP_
