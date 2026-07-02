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

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rtt/Activity.hpp"
#include "rtt/OperationCaller.hpp"
#include "rtt/RTT.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2/rtt_ros2.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"
#include "rtt_ros2_actions/rtt_action_server.hpp"
#include "rtt_ros2_actions/simple_action_server.hpp"

#include "example_interfaces/action/fibonacci.hpp"

#include "gtest/gtest.h"

using Fibonacci = example_interfaces::action::Fibonacci;
using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

static constexpr auto kTimeout = std::chrono::seconds(10);

class TestRosActionsEnvironment
  : public ::testing::Environment
{
  void SetUp() override
  {
    // Load the rosnode service and create the process-wide ROS node with its
    // own spinner thread.
    ASSERT_TRUE(rtt_ros2::import("rtt_ros2_node"));
    RTT::Service::shared_ptr global_ros =
      RTT::internal::GlobalService::Instance()->getService("ros");
    ASSERT_TRUE(global_ros);
    RTT::OperationCaller<bool()> create_node =
      global_ros->getOperation("create_node");
    ASSERT_TRUE(create_node.ready());
    ASSERT_TRUE(create_node());
    ASSERT_TRUE(rtt_ros2_node::getNode(nullptr) != nullptr);
  }
};

/**
 * Test component serving the Fibonacci action with the ROS 1 style
 * SimpleActionServer. A goal with order < 0 is rejected from the goal hook,
 * other goals preempt the active goal (acceptPending) and the sequence is
 * computed step by step in updateHook().
 */
class FibonacciServerComponent : public RTT::TaskContext
{
public:
  explicit FibonacciServerComponent(const std::string & name)
  : RTT::TaskContext(name),
    action_server_(this->provides(), "fibonacci")
  {
    action_server_.setGoalHook(
      [this](const Fibonacci::Goal & goal) {
        if (goal.order < 0) {
          Fibonacci::Result result;
          action_server_.rejectPending(result, "negative order");
          return;
        }
        Fibonacci::Result result;
        result.sequence = sequence_;
        if (action_server_.acceptPending(result, "preempted by a new goal")) {
          sequence_.clear();
          sequence_.push_back(0);
          sequence_.push_back(1);
        }
      });
    action_server_.setCancelHook(
      [this]() {
        canceled_goals_++;
      });
  }

  bool startHook() override
  {
    return action_server_.start();
  }

  void updateHook() override
  {
    if (!action_server_.isActive() && !action_server_.isPreempting()) {
      return;
    }
    auto goal = action_server_.getActiveGoal();
    if (!goal) {
      return;
    }
    if (static_cast<int32_t>(sequence_.size()) > goal->order) {
      Fibonacci::Result result;
      result.sequence = sequence_;
      action_server_.succeedActive(result, "done");
      return;
    }
    sequence_.push_back(sequence_.rbegin()[0] + sequence_.rbegin()[1]);
    Fibonacci::Feedback feedback;
    feedback.sequence = sequence_;
    action_server_.publishFeedback(feedback);
  }

  void stopHook() override
  {
    action_server_.shutdown();
  }

  rtt_ros2_actions::SimpleActionServer<Fibonacci> action_server_;
  std::vector<int32_t> sequence_;
  std::atomic<int> canceled_goals_{0};
};

class TestRosActions : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<FibonacciServerComponent>("fibonacci_server");
    server_->setActivity(new RTT::Activity(ORO_SCHED_OTHER, 0, 0.02));
    ASSERT_TRUE(server_->configure());
    ASSERT_TRUE(server_->start());

    node_ = rtt_ros2_node::getNode(nullptr);
    ASSERT_TRUE(node_ != nullptr);
    client_ = rclcpp_action::create_client<Fibonacci>(node_, "fibonacci");
    ASSERT_TRUE(client_->wait_for_action_server(kTimeout));
  }

  void TearDown() override
  {
    client_.reset();
    server_->stop();
    server_.reset();
  }

  //! Send a goal and wait until it is accepted by the server.
  FibonacciGoalHandle::SharedPtr sendGoal(
    int32_t order,
    const rclcpp_action::Client<Fibonacci>::SendGoalOptions & options =
    rclcpp_action::Client<Fibonacci>::SendGoalOptions())
  {
    Fibonacci::Goal goal;
    goal.order = order;
    auto goal_handle_future = client_->async_send_goal(goal, options);
    if (goal_handle_future.wait_for(kTimeout) != std::future_status::ready) {
      return nullptr;
    }
    return goal_handle_future.get();
  }

  //! Wait for the result of a goal.
  rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult getResult(
    FibonacciGoalHandle::SharedPtr goal_handle)
  {
    auto result_future = client_->async_get_result(goal_handle);
    if (result_future.wait_for(kTimeout) != std::future_status::ready) {
      throw std::runtime_error("timed out waiting for the action result");
    }
    return result_future.get();
  }

  std::unique_ptr<FibonacciServerComponent> server_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
};

TEST_F(TestRosActions, GoalSucceeds)
{
  auto goal_handle = sendGoal(5);
  ASSERT_TRUE(goal_handle != nullptr);

  auto result = getResult(goal_handle);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result.code);
  // sequence contains F(0)..F(order): [0, 1, 1, 2, 3, 5]
  ASSERT_EQ(6u, result.result->sequence.size());
  EXPECT_EQ(0, result.result->sequence[0]);
  EXPECT_EQ(1, result.result->sequence[1]);
  EXPECT_EQ(5, result.result->sequence[5]);
}

TEST_F(TestRosActions, GoalRejectedByHook)
{
  // The goal is deferred (emulated PENDING) and then rejected by the goal
  // hook: the client observes ABORTED, like with the ROS 1 rejectPending().
  auto goal_handle = sendGoal(-1);
  ASSERT_TRUE(goal_handle != nullptr);

  auto result = getResult(goal_handle);
  EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result.code);
  EXPECT_TRUE(result.result->sequence.empty());
}

TEST_F(TestRosActions, GoalCanceled)
{
  auto goal_handle = sendGoal(1000);
  ASSERT_TRUE(goal_handle != nullptr);

  // Let the server produce some feedback, then cancel.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(server_->action_server_.isActive());

  auto cancel_future = client_->async_cancel_goal(goal_handle);
  ASSERT_EQ(std::future_status::ready, cancel_future.wait_for(kTimeout));

  auto result = getResult(goal_handle);
  EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, result.code);
  EXPECT_EQ(1, server_->canceled_goals_.load());
  EXPECT_FALSE(server_->action_server_.isActive());
}

TEST_F(TestRosActions, GoalPreempted)
{
  auto first_goal_handle = sendGoal(1000);
  ASSERT_TRUE(first_goal_handle != nullptr);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(server_->action_server_.isActive());

  // The second goal preempts the first one (acceptPending aborts it).
  auto second_goal_handle = sendGoal(5);
  ASSERT_TRUE(second_goal_handle != nullptr);

  auto first_result = getResult(first_goal_handle);
  EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, first_result.code);

  auto second_result = getResult(second_goal_handle);
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, second_result.code);
  EXPECT_EQ(6u, second_result.result->sequence.size());
}

/**
 * Low-level RTTActionServer test with plain std::function callbacks executed
 * in the ROS thread.
 */
TEST_F(TestRosActions, RTTActionServerCallbacks)
{
  rtt_ros2_actions::RTTActionServer<Fibonacci> action_server("echo");
  std::atomic<int> goals_handled{0};

  action_server.registerAcceptedCallback(
    [&goals_handled](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> gh) {
      // The goal is already EXECUTING: the default goal callback returns
      // ACCEPT_AND_EXECUTE when an accepted handler is registered.
      auto result = std::make_shared<Fibonacci::Result>();
      result->sequence = {42};
      gh->succeed(result);
      goals_handled++;
    });

  EXPECT_EQ("echo", action_server.getName());
  EXPECT_FALSE(action_server.connected());
  ASSERT_TRUE(action_server.connect(server_.get()));
  ASSERT_TRUE(action_server.connected());

  auto client = rclcpp_action::create_client<Fibonacci>(node_, "echo");
  ASSERT_TRUE(client->wait_for_action_server(kTimeout));

  Fibonacci::Goal goal;
  goal.order = 1;
  auto goal_handle_future = client->async_send_goal(goal);
  ASSERT_EQ(std::future_status::ready, goal_handle_future.wait_for(kTimeout));
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle != nullptr);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(std::future_status::ready, result_future.wait_for(kTimeout));
  auto result = result_future.get();
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result.code);
  ASSERT_EQ(1u, result.result->sequence.size());
  EXPECT_EQ(42, result.result->sequence[0]);
  EXPECT_EQ(1, goals_handled.load());

  action_server.shutdown();
  EXPECT_FALSE(action_server.connected());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv)
  // because this function removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(
    new TestRosActionsEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
