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

#ifndef OROCOS__RTT_ROS2_ACTIONS__RTT_ACTION_SERVER_HPP_
#define OROCOS__RTT_ROS2_ACTIONS__RTT_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rtt/Logger.hpp"
#include "rtt/OperationCaller.hpp"
#include "rtt/OperationInterfacePart.hpp"
#include "rtt/TaskContext.hpp"

#include "rtt_ros2_node/rtt_ros2_node.hpp"

namespace rtt_ros2_actions
{

/**
 * @brief Thin wrapper around rclcpp_action::Server for Orocos components.
 *
 * Unlike the ROS 1 rtt_actionlib implementation there are no RTT data ports
 * and no periodic status timer: the class directly owns an
 * rclcpp_action::Server created on the node returned by
 * rtt_ros2_node::getNode() and the ROS executor of that node invokes the
 * handlers.
 *
 * Handlers can be provided in two ways:
 *
 * 1. As RTT operations (registerGoalOperation(), registerCancelOperation(),
 *    registerAcceptedOperation()). The execution thread is defined by the
 *    operation type: a ClientThread operation runs directly in the ROS
 *    executor thread, an OwnThread operation runs in the thread of the
 *    component that owns the operation. The goal and cancel operations are
 *    invoked with call semantics because their return value is required, so
 *    registering them as OwnThread operations will block the ROS executor
 *    thread until the owner's ExecutionEngine processes the call. The
 *    accepted operation is invoked with send semantics and never blocks.
 *
 * 2. As plain std::function callbacks (registerGoalCallback(),
 *    registerCancelCallback(), registerAcceptedCallback()). They always run
 *    in the ROS executor thread and take precedence over registered
 *    operations.
 *
 * Default behaviour when nothing is registered (see ACTIONLIB_TODO.md):
 *  - goal requests are accepted (ACCEPT_AND_EXECUTE) if an accepted
 *    handler is registered and rejected otherwise;
 *  - cancel requests are rejected unless a cancel handler is registered.
 */
template<class ActionT>
class RTTActionServer
{
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;
  using GoalUUID = rclcpp_action::GoalUUID;
  using GoalResponse = rclcpp_action::GoalResponse;
  using CancelResponse = rclcpp_action::CancelResponse;

  using GoalCallback = typename rclcpp_action::Server<ActionT>::GoalCallback;
  using CancelCallback = typename rclcpp_action::Server<ActionT>::CancelCallback;
  using AcceptedCallback = typename rclcpp_action::Server<ActionT>::AcceptedCallback;

  /// Signatures of the RTT operations accepted by the register*Operation() methods.
  using GoalOperationSignature =
    GoalResponse(GoalUUID, std::shared_ptr<const Goal>);
  using CancelOperationSignature = CancelResponse(GoalHandleSharedPtr);
  using AcceptedOperationSignature = void (GoalHandleSharedPtr);

  /**
   * @param name Name of the action server. Also used as the default ROS
   *             action name in connect().
   */
  explicit RTTActionServer(const std::string & name)
  : name_(name)
  {}

  ~RTTActionServer()
  {
    shutdown();
  }

  //! Get the name of the action server.
  const std::string getName() const
  {
    return name_;
  }

  /**
   * @brief Register an RTT operation as the ROS 2 GoalCallback.
   *
   * Signature: GoalResponse(GoalUUID, std::shared_ptr<const Goal>).
   * Invoked with call semantics from the ROS executor thread.
   */
  bool registerGoalOperation(RTT::OperationInterfacePart * operation)
  {
    if (!operation) {return false;}
    goal_operation_ = RTT::OperationCaller<GoalOperationSignature>(operation);
    if (!goal_operation_.ready()) {
      RTT::log(RTT::Error) << "[" << name_ << "] operation '" << operation->getName() <<
        "' is not compatible with the goal operation signature" << RTT::endlog();
      return false;
    }
    return true;
  }

  /**
   * @brief Register an RTT operation as the ROS 2 CancelCallback.
   *
   * Signature: CancelResponse(GoalHandleSharedPtr).
   * Invoked with call semantics from the ROS executor thread.
   */
  bool registerCancelOperation(RTT::OperationInterfacePart * operation)
  {
    if (!operation) {return false;}
    cancel_operation_ = RTT::OperationCaller<CancelOperationSignature>(operation);
    if (!cancel_operation_.ready()) {
      RTT::log(RTT::Error) << "[" << name_ << "] operation '" << operation->getName() <<
        "' is not compatible with the cancel operation signature" << RTT::endlog();
      return false;
    }
    return true;
  }

  /**
   * @brief Register an RTT operation as the ROS 2 AcceptedCallback.
   *
   * Signature: void(GoalHandleSharedPtr).
   * Invoked with send semantics (never blocks the ROS executor thread).
   */
  bool registerAcceptedOperation(RTT::OperationInterfacePart * operation)
  {
    if (!operation) {return false;}
    accepted_operation_ = RTT::OperationCaller<AcceptedOperationSignature>(operation);
    if (!accepted_operation_.ready()) {
      RTT::log(RTT::Error) << "[" << name_ << "] operation '" << operation->getName() <<
        "' is not compatible with the accepted operation signature" << RTT::endlog();
      return false;
    }
    return true;
  }

  //! Register a plain callback executed in the ROS thread (overrides the goal operation).
  void registerGoalCallback(GoalCallback callback)
  {
    goal_callback_ = std::move(callback);
  }

  //! Register a plain callback executed in the ROS thread (overrides the cancel operation).
  void registerCancelCallback(CancelCallback callback)
  {
    cancel_callback_ = std::move(callback);
  }

  //! Register a plain callback executed in the ROS thread (overrides the accepted operation).
  void registerAcceptedCallback(AcceptedCallback callback)
  {
    accepted_callback_ = std::move(callback);
  }

  /**
   * @brief Create the underlying rclcpp_action::Server.
   *
   * @param owner Component used to look up the ROS node
   *              (rtt_ros2_node::getNode(owner), with fallback to the
   *              process-wide node).
   * @param action_name ROS action name; defaults to getName().
   * @return true on success.
   */
  bool connect(RTT::TaskContext * owner, const std::string & action_name = std::string())
  {
    node_ = rtt_ros2_node::getNode(owner);
    if (!node_) {
      RTT::log(RTT::Error) << "[" << name_ << "] no ROS node found. Load the rosnode " <<
        "service into the component or the global service first." << RTT::endlog();
      return false;
    }

    try {
      server_ = rclcpp_action::create_server<ActionT>(
        node_,
        action_name.empty() ? name_ : action_name,
        std::bind(
          &RTTActionServer<ActionT>::handleGoal, this,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(
          &RTTActionServer<ActionT>::handleCancel, this,
          std::placeholders::_1),
        std::bind(
          &RTTActionServer<ActionT>::handleAccepted, this,
          std::placeholders::_1));
    } catch (const std::exception & e) {
      RTT::log(RTT::Error) << "[" << name_ << "] failed to create the action server: " <<
        e.what() << RTT::endlog();
      server_.reset();
      node_.reset();
      return false;
    }
    return true;
  }

  //! Check whether the underlying rclcpp_action::Server has been created.
  bool connected() const
  {
    return static_cast<bool>(server_);
  }

  //! Destroy the underlying rclcpp_action::Server.
  void shutdown()
  {
    server_.reset();
    node_.reset();
  }

  //! Access the underlying rclcpp_action::Server (may be nullptr).
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server()
  {
    return server_;
  }

  //! Access the node the server was created on (may be nullptr).
  rclcpp::Node::SharedPtr node()
  {
    return node_;
  }

private:
  GoalResponse handleGoal(const GoalUUID & uuid, std::shared_ptr<const Goal> goal)
  {
    if (goal_callback_) {
      return goal_callback_(uuid, goal);
    }
    if (goal_operation_.ready()) {
      return goal_operation_(uuid, goal);
    }
    // Default: only accept goals which somebody is going to execute.
    if (accepted_callback_ || accepted_operation_.ready()) {
      return GoalResponse::ACCEPT_AND_EXECUTE;
    }
    RTT::log(RTT::Warning) << "[" << name_ << "] goal rejected: no accepted " <<
      "handler has been registered" << RTT::endlog();
    return GoalResponse::REJECT;
  }

  CancelResponse handleCancel(GoalHandleSharedPtr goal_handle)
  {
    if (cancel_callback_) {
      return cancel_callback_(goal_handle);
    }
    if (cancel_operation_.ready()) {
      return cancel_operation_(goal_handle);
    }
    // Default: cancellation is not supported.
    return CancelResponse::REJECT;
  }

  void handleAccepted(GoalHandleSharedPtr goal_handle)
  {
    if (accepted_callback_) {
      accepted_callback_(goal_handle);
      return;
    }
    if (accepted_operation_.ready()) {
      // send() so that an OwnThread operation does not block the ROS thread
      accepted_operation_.send(goal_handle);
    }
  }

  std::string name_;
  rclcpp::Node::SharedPtr node_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;

  RTT::OperationCaller<GoalOperationSignature> goal_operation_;
  RTT::OperationCaller<CancelOperationSignature> cancel_operation_;
  RTT::OperationCaller<AcceptedOperationSignature> accepted_operation_;

  GoalCallback goal_callback_;
  CancelCallback cancel_callback_;
  AcceptedCallback accepted_callback_;
};

}  // namespace rtt_ros2_actions

#endif  // OROCOS__RTT_ROS2_ACTIONS__RTT_ACTION_SERVER_HPP_
