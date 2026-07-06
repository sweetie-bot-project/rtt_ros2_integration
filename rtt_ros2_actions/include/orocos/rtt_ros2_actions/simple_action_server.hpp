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

#ifndef OROCOS__RTT_ROS2_ACTIONS__SIMPLE_ACTION_SERVER_HPP_
#define OROCOS__RTT_ROS2_ACTIONS__SIMPLE_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "boost/make_shared.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "rtt/Logger.hpp"
#include "rtt/Operation.hpp"
#include "rtt/OperationCaller.hpp"
#include "rtt/Service.hpp"
#include "rtt/TaskContext.hpp"

#include "rtt_ros2_actions/rtt_action_server.hpp"

namespace rtt_ros2_actions
{

/**
 * @brief ROS 1 sweetie_bot::OrocosSimpleActionServer compatibility layer.
 *
 * Emulates the ROS 1 actionlib PENDING state on top of rclcpp_action:
 * every incoming goal is accepted with ACCEPT_AND_DEFER and stored in the
 * pending slot, then the goal hook is invoked. The component implementation
 * makes the goal active with acceptPending() (which aborts the previously
 * active goal) or discards it with rejectPending() (the client observes
 * ABORTED). isPreempting() maps to the ROS 2 CANCELING state.
 *
 * Threading: the goal and cancel hooks are dispatched through OwnThread RTT
 * operations registered in a sub-service named after the action server, so
 * user code always runs in the thread of the owning component, like in the
 * ROS 1 implementation. The owner therefore must have an active
 * ExecutionEngine (an Activity) for the hooks to fire. All public methods
 * are thread-safe.
 *
 * Differences from ROS 1 dictated by the rclcpp_action state machine:
 *  - cancelActive() finishes the goal as CANCELED only if the client
 *    requested cancellation (CANCELING state); otherwise the goal is
 *    finished as ABORTED, because ROS 2 has no server-side cancel
 *    transition.
 *  - Result messages carry no status text; the msg arguments are logged
 *    only.
 */
template<class ActionT>
class SimpleActionServer
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

  using GoalHook = std::function<void (const Goal &)>;
  using CancelHook = std::function<void ()>;

  /**
   * @brief Create the server and its hook operations.
   *
   * A sub-service with the name @p name is created in @p owner_service and
   * the internal handlers are added to it as operations
   * ("goalCallback", "cancelCallback", "acceptedCallback", "cancelHandler"),
   * so they are visible for introspection and are registered with the
   * underlying RTTActionServer. The accepted/cancel handlers are OwnThread
   * operations: user hooks run in the owner component thread.
   *
   * @param owner_service Service of the owning component.
   * @param name Name of the action server, of the sub-service and the
   *             default ROS action name.
   * @throw std::invalid_argument if owner_service is null or not owned by
   *        a TaskContext.
   */
  SimpleActionServer(boost::shared_ptr<RTT::Service> owner_service, const std::string & name)
  : action_server_(name),
    goal_operation_("goalCallback"),
    cancel_operation_("cancelCallback"),
    accepted_operation_("acceptedCallback"),
    cancel_handler_operation_("cancelHandler")
  {
    if (!owner_service) {
      throw std::invalid_argument("SimpleActionServer: owner service pointer must be valid");
    }
    owner_ = owner_service->getOwner();
    if (!owner_) {
      throw std::invalid_argument(
              "SimpleActionServer: owner service must belong to a TaskContext");
    }

    // Sub-service named after the action server holding the handler operations.
    service_ = boost::make_shared<RTT::Service>(name, owner_);
    owner_service->addService(service_);

    // Lightweight check executed directly in the ROS executor thread.
    goal_operation_.calls(
      boost::function<GoalResponse(GoalUUID, std::shared_ptr<const Goal>)>(
        [this](GoalUUID uuid, std::shared_ptr<const Goal> goal) {
          return this->goalCallback(uuid, goal);
        }),
      RTT::ClientThread);
    cancel_operation_.calls(
      boost::function<CancelResponse(GoalHandleSharedPtr)>(
        [this](GoalHandleSharedPtr gh) {
          return this->cancelCallback(gh);
        }),
      RTT::ClientThread);
    // User code runs in the owner component thread.
    accepted_operation_.calls(
      boost::function<void(GoalHandleSharedPtr)>(
        [this](GoalHandleSharedPtr gh) {
          this->acceptedCallback(gh);
        }),
      RTT::OwnThread, owner_->engine());
    cancel_handler_operation_.calls(
      boost::function<void(GoalHandleSharedPtr)>(
        [this](GoalHandleSharedPtr gh) {
          this->cancelHandler(gh);
        }),
      RTT::OwnThread, owner_->engine());

    service_->addOperation(goal_operation_)
    .doc("Checks a new (not yet accepted) goal. Executed in the ROS thread.");
    service_->addOperation(cancel_operation_)
    .doc("Accepts cancel requests for known goals. Executed in the ROS thread.");
    service_->addOperation(accepted_operation_)
    .doc("Stores the accepted goal in the pending slot and calls the goal hook.");
    service_->addOperation(cancel_handler_operation_)
    .doc("Finishes cancellation of a goal and calls the cancel hook.");

    action_server_.registerGoalOperation(service_->getOperation("goalCallback"));
    action_server_.registerCancelOperation(service_->getOperation("cancelCallback"));
    action_server_.registerAcceptedOperation(service_->getOperation("acceptedCallback"));

    cancel_handler_caller_ =
      RTT::OperationCaller<void(GoalHandleSharedPtr)>(
      service_->getOperation("cancelHandler"));
  }

  ~SimpleActionServer()
  {
    shutdown();
  }

  //! Get the name of the action server.
  const std::string getName() const
  {
    return action_server_.getName();
  }

  //! Check whether the server is connected to ROS.
  bool ready()
  {
    return action_server_.connected();
  }

  /**
   * @brief Set the ROS action name advertised by start().
   *
   * Supports the usual ROS name expansion, e.g. "~/controller/joint_state"
   * resolves relative to the node name. Must be called before start();
   * an empty string (default) means the server name passed to the
   * constructor is used as-is.
   */
  void setActionName(const std::string & action_name)
  {
    action_name_ = action_name;
  }

  /**
   * @brief Create the ROS action server.
   * @param publish_feedback Unused, kept for interface compatibility with
   *        the ROS 1 implementation (feedback is only published explicitly
   *        with publishFeedback()).
   */
  bool start(bool publish_feedback = false)
  {
    if (publish_feedback) {
      RTT::log(RTT::Warning) << "[" << getName() << "] periodic feedback " <<
        "publishing is not implemented in the ROS 2 version" << RTT::endlog();
    }
    if (action_server_.connected()) {
      return true;
    }
    return action_server_.connect(owner_, action_name_);
  }

  //! Disconnect from ROS. Active and pending goals are aborted by rclcpp_action.
  void shutdown()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    goal_active_.reset();
    goal_pending_.reset();
    action_server_.shutdown();
  }

  //! Check if an active (executing) goal is present.
  bool isActive() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return goal_active_ && goal_active_->is_executing();
  }

  //! Check if the active goal is being canceled by the client (ROS 1 PREEMPTING).
  bool isPreempting() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return goal_active_ && goal_active_->is_canceling();
  }

  //! Alias with the historical sweetie_bot spelling.
  bool isPreemting() const
  {
    return isPreempting();
  }

  //! Check if a pending (deferred, ROS 1 PENDING) goal is present.
  bool isPending() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return static_cast<bool>(pendingGoalHandle());
  }

  //! Return the active goal or nullptr.
  std::shared_ptr<const Goal> getActiveGoal() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (goal_active_ && (goal_active_->is_executing() || goal_active_->is_canceling())) {
      return goal_active_->get_goal();
    }
    return nullptr;
  }

  //! Return the pending goal or nullptr.
  std::shared_ptr<const Goal> getPendingGoal() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GoalHandleSharedPtr pending = pendingGoalHandle();
    return pending ? pending->get_goal() : nullptr;
  }

  /**
   * @brief Accept the pending goal.
   *
   * The pending goal becomes active (EXECUTING). The previously active goal,
   * if any, is finished with ABORTED (or CANCELED if the client requested
   * cancellation) and the given result.
   *
   * @param result Result of the finished previously active goal.
   * @return true if the pending goal was accepted.
   */
  bool acceptPending(const Result & result, const std::string & msg = "")
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GoalHandleSharedPtr pending = pendingGoalHandle();
    if (!pending) {
      return false;
    }
    finishActive(result, msg, false);
    try {
      pending->execute();
    } catch (const std::exception & e) {
      RTT::log(RTT::Error) << "[" << getName() << "] acceptPending: " << e.what() <<
        RTT::endlog();
      goal_pending_.reset();
      return false;
    }
    goal_active_ = pending;
    goal_pending_.reset();
    return true;
  }

  /**
   * @brief Reject the pending goal.
   *
   * The rclcpp_action state machine has no reject transition for an accepted
   * goal, so the goal is shortly executed and finished with ABORTED and the
   * given result (the client observes ABORTED), as designed in
   * ACTIONLIB_TODO.md.
   *
   * @param result Result of the rejected pending goal.
   * @return true if the pending goal was rejected.
   */
  bool rejectPending(const Result & result, const std::string & msg = "")
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    GoalHandleSharedPtr pending = pendingGoalHandle();
    if (!pending) {
      return false;
    }
    logResult("rejectPending", msg);
    terminateGoal(pending, result);
    goal_pending_.reset();
    return true;
  }

  //! Publish feedback on the active goal. @return true if an active goal is present.
  bool publishFeedback(const Feedback & feedback)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!goal_active_ || !(goal_active_->is_executing() || goal_active_->is_canceling())) {
      return false;
    }
    goal_active_->publish_feedback(std::make_shared<Feedback>(feedback));
    return true;
  }

  //! Abort the active goal with the given result.
  bool abortActive(const Result & result, const std::string & msg = "")
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    logResult("abortActive", msg);
    return finishActive(result, msg, false);
  }

  /**
   * @brief Cancel the active goal with the given result.
   *
   * If the client requested cancellation (CANCELING state) the goal is
   * finished as CANCELED; otherwise ROS 2 does not allow a server-side
   * cancel and the goal is finished as ABORTED.
   */
  bool cancelActive(const Result & result, const std::string & msg = "")
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    logResult("cancelActive", msg);
    return finishActive(result, msg, true);
  }

  //! Succeed the active goal with the given result.
  bool succeedActive(const Result & result, const std::string & msg = "")
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!goal_active_ || !(goal_active_->is_executing() || goal_active_->is_canceling())) {
      return false;
    }
    logResult("succeedActive", msg);
    try {
      goal_active_->succeed(std::make_shared<Result>(result));
    } catch (const std::exception & e) {
      RTT::log(RTT::Error) << "[" << getName() << "] succeedActive: " << e.what() <<
        RTT::endlog();
      return false;
    }
    goal_active_.reset();
    return true;
  }

  /**
   * @brief Set the new goal hook.
   *
   * Called in the owner component thread whenever a new goal has been
   * deferred into the pending slot.
   */
  void setGoalHook(GoalHook hook)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    new_goal_hook_ = std::move(hook);
  }

  /**
   * @brief Set the cancel hook.
   *
   * Called in the owner component thread when the client cancels the active
   * goal, before the goal is automatically finished as CANCELED.
   */
  void setCancelHook(CancelHook hook)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    cancel_goal_hook_ = std::move(hook);
  }

private:
  //! Return the pending goal handle if it is still deferred (not finished, not canceling).
  GoalHandleSharedPtr pendingGoalHandle() const
  {
    if (goal_pending_ && goal_pending_->is_active() &&
      !goal_pending_->is_executing() && !goal_pending_->is_canceling())
    {
      return goal_pending_;
    }
    return nullptr;
  }

  void logResult(const char * operation, const std::string & msg)
  {
    if (!msg.empty()) {
      RTT::log(RTT::Debug) << "[" << getName() << "] " << operation << ": " << msg <<
        RTT::endlog();
    }
  }

  /**
   * Finish the active goal: CANCELED if it is in the CANCELING state,
   * ABORTED otherwise (also when @p prefer_canceled is false).
   * @return true if the goal was finished.
   */
  bool finishActive(const Result & result, const std::string & msg, bool prefer_canceled)
  {
    (void)msg;
    if (!goal_active_) {
      return false;
    }
    if (!(goal_active_->is_executing() || goal_active_->is_canceling())) {
      goal_active_.reset();
      return false;
    }
    try {
      if (goal_active_->is_canceling()) {
        goal_active_->canceled(std::make_shared<Result>(result));
      } else {
        if (prefer_canceled) {
          RTT::log(RTT::Warning) << "[" << getName() << "] cancelActive: no client " <<
            "cancel request, the goal is finished as ABORTED" << RTT::endlog();
        }
        goal_active_->abort(std::make_shared<Result>(result));
      }
    } catch (const std::exception & e) {
      RTT::log(RTT::Error) << "[" << getName() << "] failed to finish the active goal: " <<
        e.what() << RTT::endlog();
      goal_active_.reset();
      return false;
    }
    goal_active_.reset();
    return true;
  }

  //! Finish a deferred goal as ABORTED (or CANCELED when it is being canceled).
  void terminateGoal(const GoalHandleSharedPtr & gh, const Result & result)
  {
    try {
      if (gh->is_canceling()) {
        gh->canceled(std::make_shared<Result>(result));
        return;
      }
      if (!gh->is_executing()) {
        gh->execute();
      }
      gh->abort(std::make_shared<Result>(result));
    } catch (const std::exception & e) {
      RTT::log(RTT::Error) << "[" << getName() << "] failed to terminate a goal: " <<
        e.what() << RTT::endlog();
    }
  }

  //! ROS 2 GoalCallback: every goal is deferred (emulated ROS 1 PENDING state).
  GoalResponse goalCallback(const GoalUUID & /*uuid*/, std::shared_ptr<const Goal>/*goal*/)
  {
    return GoalResponse::ACCEPT_AND_DEFER;
  }

  //! ROS 2 CancelCallback: accept and forward to the OwnThread cancel handler.
  CancelResponse cancelCallback(GoalHandleSharedPtr gh)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (gh == goal_active_ || gh == goal_pending_) {
      // The CANCELING transition happens after this callback returns; the
      // actual handling runs in the owner thread (see cancelHandler()).
      cancel_handler_caller_.send(gh);
      return CancelResponse::ACCEPT;
    }
    return CancelResponse::REJECT;
  }

  //! ROS 2 AcceptedCallback (OwnThread): defer the goal and call the goal hook.
  void acceptedCallback(GoalHandleSharedPtr gh)
  {
    GoalHook hook;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      GoalHandleSharedPtr replaced = pendingGoalHandle();
      if (replaced) {
        terminateGoal(replaced, Result());
        RTT::log(RTT::Debug) << "[" << getName() << "] pending goal is replaced " <<
          "by a new received goal" << RTT::endlog();
      }
      goal_pending_ = gh;
      hook = new_goal_hook_;
    }
    if (hook) {
      hook(*gh->get_goal());
    }
  }

  //! OwnThread continuation of cancelCallback(); the goal is in the CANCELING state.
  void cancelHandler(GoalHandleSharedPtr gh)
  {
    CancelHook hook;
    bool is_active = false;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (gh == goal_active_) {
        is_active = true;
        hook = cancel_goal_hook_;
      } else if (gh == goal_pending_) {
        terminateGoal(gh, Result());
        goal_pending_.reset();
        return;
      } else {
        // The goal was finished or replaced before the handler ran.
        return;
      }
    }
    if (hook) {
      hook();
    }
    // ROS 1 compatibility: finish the goal unless the hook already did.
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (gh == goal_active_) {
      finishActive(Result(), "active goal is canceled by client request", true);
    }
  }

  RTTActionServer<ActionT> action_server_;
  RTT::TaskContext * owner_ = nullptr;
  boost::shared_ptr<RTT::Service> service_;
  std::string action_name_;

  RTT::Operation<GoalResponse(GoalUUID, std::shared_ptr<const Goal>)> goal_operation_;
  RTT::Operation<CancelResponse(GoalHandleSharedPtr)> cancel_operation_;
  RTT::Operation<void(GoalHandleSharedPtr)> accepted_operation_;
  RTT::Operation<void(GoalHandleSharedPtr)> cancel_handler_operation_;
  RTT::OperationCaller<void(GoalHandleSharedPtr)> cancel_handler_caller_;

  mutable std::recursive_mutex mutex_;
  GoalHandleSharedPtr goal_active_;
  GoalHandleSharedPtr goal_pending_;
  GoalHook new_goal_hook_;
  CancelHook cancel_goal_hook_;
};

}  // namespace rtt_ros2_actions

#endif  // OROCOS__RTT_ROS2_ACTIONS__SIMPLE_ACTION_SERVER_HPP_
