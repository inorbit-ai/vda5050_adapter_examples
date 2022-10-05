## VDA5050 Template package (C++)

ROS2 package with a C++ template to implement a custom VDA5050 adapter.

This package contains:

- **config**: Pre-configure yaml file that loads the handler plugins, as well as nodes configuration parameters. Use this file as a guide to define handler class names, supported actions, etc.
- **launch**: Two `*.launch.py` files, one for launching the `adapter_node` alone, and the other to launch the whole connector (mqtt_bridge, controller and adapter).
- **plugins**: The package provides three plugin template files, one for each kind of handler (`state`, `NavToNode` and `VDAAction`), with their corresponding headers under the include folder.

## How to define plugins

To define a new plugin for a given handler kind, you need to:

- Create its cpp file under the plugins folder.
- Define the pure virtual functions (see sections below for each handler).
- Register the plugin with the macro `PLUGINLIB_EXPORT_CLASS(derivedClass, baseClass)` on the header file.
- Add the file to the CMakeList under `plugins SET`.
- Register the new plugin class on the `plugins.xml` file.
- Add the corresponding class name and parameters information to the `config/connector.yaml` file for the adapter to load it.

### State handler

The VDA state information is shared by the adapter through its state handlers. NavToNode and VDAAction handlers also populate these fields by sharing the updates of the commands they handle.

The state handler plugins populate the `current_state` with robot information like Odometry, BatteryStatus or errors / warnings. 

To add a new state handler, you will need to define the following functions:

- **configure**: Used to define ROS2 interfaces (topics, services), initialize handler vars, etc. It is called at start time only.
- **execute**: Used to modify a specific field within the adapter's global `order_state`. When the controller requests the adapter `order_state` info, it goes to every state handler requesting each to update their specific information. See how to modify [global order_state](#modify-global-order_state) for more details.

### NavToNode handler

Used by the adapter to send a goal request to the robot's navigation stack. This handler should also update the `driving` status under the [global order_state](#modify-global-order_state), since later this status is used to either accept or reject new goal requests coming from the controller.

To define your navigate to node plugin, define the following functions:

- **configure**: Used to define ROS2 interfaces (topics, services), initialize handler vars, etc. It is called at start time only.
- **execute**: Used to define the process to send a navigation goal request to the robot's API. When the adapter accepts a navigation goal coming from the controller, it passes the _goal_handle_ instance to this handler which is used to report feedback and result status (finished or failed).
- **cancel**: Should be used to try to cancel an active navigation goal and report if it was possible.

### VDA Action handler

Used by the adapter to execute VDA Actions requests coming from the controller. These plugins need to be defined to execute a specific robot task (localize, lift a load, trigger a warning alarm, etc).

**Note**: There are three default VDA Actions already implemented on the **controller**: _cancelOrder_, _stateRequest_ and _factsheetRequest_.

To define a new VDA Action, define the following functions:

- **configure**: Used to define ROS2 interfaces (topics, services), initialize handler vars, etc. It is called at start time only.
- **cancel**: Should be used to try to cancel an active action and report if it was possible.
- **initialize, run, pause, finish, fail**: These define what to do on each step of the state machine handling VDA Actions (see [Fig14](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md#-611-actionstates)). The transition between these states is flagged through a local STATE variable which you will need to update accordingly using the method **update_action_state(STATE)**.

  We strongly recommend to implement at least the _initialize_, _finish_ and _fail_ functions. The first one is always being called (handles the transition WAITING -> INITIALIZING), and is expected to update the STATE of the action. The last two functions are the ones that stop the execution of the state machine, and any action must fall into either of these states.

  Please be extra careful to avoid creating self-blocking states (infinite action execution).

### Global order state

The adapter creates and shares a global `order_state` (unique) to update robot-specific information. This state can be accessed and modified by any handler, on any runtime moment, using the setter and getter functions defined for it.

- To add a new array field (load, error or general information), use the `add_*()` methods (e.g. `add_error(vda5050_msgs/Error)`). To clear an array field, use the `clear()` method.
- To set a value on any state field, use the `set_parameter` method. This method requires the member type address to modify, and its value. For example:

  ```cp
  current_state->set_parameter(&vda5050_msgs::msg::OrderState::driving, true);
  ```

  ```cp
  vda5050_msgs::msg::BatteryStatus battery;
  ...
  current_state->set_parameter(&vda5050_msgs::msg::OrderState::BatteryStatus, battery);
  ```

- Use the `get()` method to get a copy of the order state, or `reset()` to wipe out the entire state.