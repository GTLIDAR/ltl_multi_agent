# ltl_automaton_hil_mic

## Overview
A package providing HIL (Human-In-the-Loop) MIC (Mix-Initiative Controllers) to be used with the ltl_automaton_planner. The package provides so far two controllers: one for velocity command (geometry_msgs/Twist) and one for a "boolean" command.

## Launch files
- **bool_hil_mic_test.launch**: Example launch file for the boolean HIL MIC node implementation.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `false`.
    - `agent_name` Agent name. Default: `nexus`.


## Nodes
### vel_cmd_mix_initiative_controller.py
Mix human and planner input according to the distance with "trap" regions. Trap regions are region of either "2d_pose_region", "3d_pose_region", "2d_point_region" or "3d_point_region" TS types which would make the LTL formula impossible to fulfill if reached.

#### Subscribed Topics

- `ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic. The agent TS state is composed of a list of states from the different state models composing the action model. The planner node receives the agent TS state on this topic and update accordingly the next action and the set of possible states.

- `key_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

    Human teleop velocity command input to be mixed with the planner velocity command input.

- `nav_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

    Velocity command from the low level navigation, to be mixed with the human velocity command input.

#### Published Topics

- `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

    Mixed velocity output of the controller.

#### Used Services

- `closest_region` ([ltl_automaton_msgs/ClosestState](/ltl_automaton_msgs/srv/ClosestState.srv))

    Get the closest region (2d_pose_region) and distance to said region.

- `check_for_trap` ([ltl_automaton_msgs/TrapCheck](/ltl_automaton_msgs/srv/TrapCheck.srv))

    Check if sent state is a trap state and if is connected to current TS state.

#### Parameters

- `~ds` (float, default: 1.2)

    Safety distance. If the distance to the trap region is smaller than the safety distance, velocity command output is navigation command only.

- `~epsilon` (float, default: 1.5)

    Buffer distance. If the distance to the trap region is in between safety distance and safety distance plus epsilon, velocity command output is a mix of human command and navigation command.

- `~deadband` (float, default: 0.2)

    Human inputs are only considered if ||linear_velocity|| or ||angular_velocity|| is greater than the deadband.

- `~max_linear_x_vel` (float, default: 0.5)

    Maximum allowed value for the human command linear velocity on the x-axis.

- `~max_linear_y_vel` (float, default: 0.5)

    Maximum allowed value for the human command linear velocity on the y-axis.

- `~max_linear_z_vel` (float, default: 0.5)

    Maximum allowed value for the human command linear velocity on the z-axis.

- `~max_angular_x_vel` (float, default: 2.0)

    Maximum allowed value for the human command angular velocity on the x-axis.

- `~max_angular_y_vel` (float, default: 2.0)

    Maximum allowed value for the human command angular velocity on the y-axis.

- `~max_angular_z_vel` (float, default: 2.0)

    Maximum allowed value for the human command angular velocity on the z-axis.

- `~timeout` (float, default: 0.2)

    Human inputs are not considered if last input is older than the timeout value.

- `~node_frequency` (float, default: 50)

    Node frequency (and therefor mixed velocity command ouput frequency).

- `state_dimension_name` (string, default: "2d_pose_region")
    
    Type of state dimension to track for traps. Supported types are "2d_pose_region", "3d_pose_region", "2d_point_region" and "3d_point_region".

### bool_cmd_mix_initiative_controller.py
Mix human and planner commands for action triggered by a change of value on a boolean topic. Human commands would only be executed if execution won't make agent enter a "trap" state. A "trap" state is a TS state that would make the LTL formula impossible to fulfill if reached. Planner commands are always carried out.

#### Subscribed Topics

- `ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic. The agent TS state is composed of a list of states from the different state models composing the action model. The planner node receives the agent TS state on this topic and update accordingly the next action and the set of possible states.

- `key_cmd` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Human command input to be executed or not depending on trap states.

- `planner_cmd` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Planner command input, will always be executed.

#### Published Topics

- `mix_cmd` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Mix command output from the controller.

#### Used Services

- `check_for_trap` ([ltl_automaton_msgs/TrapCheck](/ltl_automaton_msgs/srv/TrapCheck.srv))

    Check if sent state is a trap state and if is connected to current TS state.

#### Parameters

- `transition_system_textfile` (string)

    Action model transition system definition.

- `~state_dimension_name` (string, default: "load")

    TS state dimension to track.

- `~monitored_action` (string, default: "pick")

    Action (from the transition system, and acting on the tracked state dimension) that would be executed when the output topic is switched to "True"

## Plugins
In addition of the controller nodes, the package provides the plugins (run at ltl_automaton_planner/planner_node.py level) necessary for those controllers to work. For more information on the plugins, please take a look at the [wiki](../../../wiki/Planner-Plugin).

### trap_detection.py
Provides the trap check service in the LTL planner node. The plugin uses the requested TS state to update its LTL state and check if this updated LTL state would be able to fulfill the given LTL formula. If not, the tested TS state is considered a trap.

#### Services

- `check_for_trap` ([ltl_automaton_msgs/TrapCheck](/ltl_automaton_msgs/srv/TrapCheck.srv))

    Check if sent state is a trap state and if is connected to current TS state.

### inverse_reinforcement_learning.py
Implement a IRL (Inverse Reinforcement Learning) of the expression of the LTL soft task. The plugin will start recording a run of TS states when teaching is triggered. When learning is triggered, the plugin will use this teaching run to adjust the design parameter gamma and replan. A teaching run where the soft task has been fulfilled (usually through the intervention of human inputs) will result in an increased gamma and the soft task being part of the new plan. On the other hand, a teaching run without the soft task being fulfilled will result in a decreased gamma and the soft task not being part of the new plan.

#### Subscribed Topics

- `irl_trigger` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Will trigger start of recording of teaching run when switched to True. Once recording of teaching has started, receiving a False message will stop recording and trigger learning using the recorded teaching run.

#### Published Topics

- `possible_runs`  ([ltl_automaton_msgs/LTLStateRuns](/ltl_automaton_msgs/msg/LTLStateRuns.msg))
    
    Current recorded teaching runs. For debug purposes only.

#### Arguments

- `max_run_buffer_size` (integer, default: 100)

	Maximum number of states allowed in the recorded teaching run. If number of state in the teaching run exceeds this value, recorded is stopped and learning is triggered.

