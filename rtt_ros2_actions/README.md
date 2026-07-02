# rtt_ros2_actions

ROS 2 action server support for Orocos RTT components.

## Classes

### `rtt_ros2_actions::RTTActionServer<ActionT>`

A thin wrapper around `rclcpp_action::Server` (header
`rtt_ros2_actions/rtt_action_server.hpp`). There are no RTT data ports and no
periodic status timer (unlike the ROS 1 `rtt_actionlib` implementation): the
server is created on the node returned by `rtt_ros2_node::getNode()` and its
handlers are invoked by the ROS executor.

Handlers can be registered either as RTT operations
(`registerGoalOperation()`, `registerCancelOperation()`,
`registerAcceptedOperation()`) — then the operation type (`ClientThread` /
`OwnThread`) defines the execution thread — or as plain `std::function`
callbacks executed in the ROS thread. The accepted operation is invoked with
*send* semantics so an `OwnThread` operation never blocks the ROS executor.

Default behaviour without registered handlers: goals are rejected unless an
accepted handler exists, cancel requests are rejected.

```cpp
rtt_ros2_actions::RTTActionServer<example_interfaces::action::Fibonacci> server{"fibonacci"};
server.registerGoalCallback(...);      // optional
server.registerAcceptedCallback(...);  // required to accept goals
server.connect(this);                  // inside configureHook()
```

### `rtt_ros2_actions::SimpleActionServer<ActionT>`

A compatibility layer (header `rtt_ros2_actions/simple_action_server.hpp`)
with the semantics of the ROS 1 `sweetie_bot::OrocosSimpleActionServer`:

* every incoming goal is accepted as `ACCEPT_AND_DEFER` (the emulated ROS 1
  *PENDING* state) and the goal hook fires;
* `acceptPending()` aborts the currently active goal and makes the pending
  goal active; `rejectPending()` finishes the pending goal as ABORTED;
* `isPreempting()` maps to the ROS 2 *CANCELING* state; when the client
  cancels the active goal the cancel hook fires and the goal is finished as
  CANCELED (ROS 1 auto-cancel behaviour);
* `abortActive()` / `cancelActive()` / `succeedActive()` / `publishFeedback()`
  keep their ROS 1 meaning.

The hooks are dispatched through `OwnThread` RTT operations registered in a
sub-service named after the action server, so user code always runs in the
thread of the owning component. The component must run an Activity, otherwise
the hooks never fire.

```cpp
class MyController : public RTT::TaskContext {
  rtt_ros2_actions::SimpleActionServer<MyAction> action_server_;

 public:
  MyController(const std::string & name)
  : TaskContext(name), action_server_(this->provides(), "my_action")
  {
    action_server_.setGoalHook([this](const MyAction::Goal & goal) { ... });
    action_server_.setCancelHook([this]() { ... });
  }

  bool startHook() override { return action_server_.start(); }
};
```

Known deviations from ROS 1 (dictated by the `rclcpp_action` state machine):

* `cancelActive()` finishes the goal as CANCELED only when the client
  requested cancellation, otherwise as ABORTED (ROS 2 has no server-side
  cancel transition);
* result messages carry no status text — the `msg` arguments are logged only;
* a full `actionlib::ActionServer` compatible interface is not provided: the
  `GoalHandle` interface changed incompatibly in ROS 2.
