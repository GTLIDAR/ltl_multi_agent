# ltl_automaton_planner

## Overview
An LTL (Linear Temporal Logic) planner implementation based on LTL2BA and LTL automata. Takes as input a set of action models to build a transition system, and a LTL formula (with hard and soft components).

A non-deterministic state graph, called product graph, is generated from the product of the Buchi automaton and the action model. This product graph is used to track possible agent state and generate a plan (output word) and action sequence (input word) for the agent.

For more information, please take a look at the [wiki](../../../wiki)

## Config files
- **example_ltl_formula.yaml** Example of LTL formula with both hard and soft task.

- **example_ts.yaml** Example of LTL transition system definition.

## Launch files
- **ltl_planner.launch**: Run the LTL planner node. Need a transition system definition text parameter and a LTL formula parameter.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.

- **ltl_planner_example.launch**: Example of the LTL planner implementation. Run the planner node with an example TS (Transition System) and example LTL formula.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.
    - `agent_name` Agent name. Default: `turtlebot`.

## Nodes
### planner_node.py
Planner node. Build a product graph from a given transition system and LTL formula. Uses the graph to generate a run and an action plan to follow to satisfy the formula. The planner keep track of possible states by receiving TS (Transition System) state from the agent and output action command to the agent.

#### Subscribed Topics
- `ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic. The agent TS state is composed of a list of states from the different state models composing the action model. The planner node receives the agent TS state on this topic and update accordingly the next action and the set of possible states.

#### Published Topics
- `next_move_cmd` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Next move from the output word (action sequence) to be carried out by the agent in order to satisfy the plan.

- `possible_ltl_states` ([ltl_automaton_msgs/LTLStateArray](/ltl_automaton_msgs/msg/LTLStateArray.msg))
    
    Current possible states of the agent, can be more than one as the system is non-deterministic. LTL states are composed of a TS (Transisition System) state and a Büchi state.

- `prefix_plan` ([ltl_automaton_msgs/LTLPlan](/ltl_automaton_msgs/msg/LTLPlan.msg))

    Prefix plan (also called prefix word), the action sequence to be carried out once by the agent after planning.

- `suffix_plan` ([ltl_automaton_msgs/LTLPlan](/ltl_automaton_msgs/msg/LTLPlan.msg))
    
    Suffix plan (also called suffix word), the action sequence to be carried out repeatively after the prefix plan.
    
#### Services

- `replanning` ([ltl_automaton_msgs/TaskPlanning](/ltl_automaton_msgs/srv/TaskPlanning.srv))
    
    Triggers planning to satisfy the requested hard and soft task.

#### Parameters
- `agent_name` (string, default: "agent")

    Agent name.
    
- `hard_task` (string)

    Hard task to be carried out by the agent, is required. Should be written using syntax from [LTL2BA](http://www.lsv.fr/~gastin/ltl2ba/). It is advised to use a YAML file to define this parameter as some characters in LTL syntax might cause trouble with XML in launch files. More information on the syntax on the [wiki page](../../../wiki/LTL-Formula).

- `soft_task` (string)

    Soft task to be carried out by the agent, is optional. Soft tasks are enforced or not by the planner depending on the beta value. Should be written using syntax from [LTL2BA](http://www.lsv.fr/~gastin/ltl2ba/). It is advised to use a YAML file to define this parameter as some characters in LTL syntax might cause trouble with XML in launch files. More information on the syntax on the [wiki page](../../../wiki/LTL-Formula).

- `initial_beta` (double, default: 1000)

    Affect how the soft task is enforced by the planner. A higher value will help enforcing the soft task while a lower value can leave the soft task unenforced by the plan. The given value is only the initial one since beta can vary.

- `gamma` (double, default: 10)

    Suffix weighting parameter. A higher value will increase the suffix cost and therefor minimize suffix word length.

- `transition_system_textfile` (string)

    Action model transition system definition. More information on the action model can be found below.


- `~check_timestamp` (boolean, default: true)
    
    If false, ignore a received TS state message when the timestamp is identical to previously received TS state message timestamp.


- `~initial_ts_state_from_agent` (boolean, default: false)

    If true, will wait for the TS state from agent to build initial state in product graph. If false, will use initial states from transition system definition.


- `~replan_on_unplanned_move` (boolean, default: true)
    
    If true, will replan when receiving a TS state that is not the next one in the plan (output word) using the TS state as initial state.
    
#### LTL formula
For more information on LTL task formulation and syntax, please take a look at the [wiki page](../../../wiki/LTL-Formula)

#### Transition system definition
The final transition system is built from one or more action models. Those action models are also transition systems and their graph product is the final transition system.

As the final transition system, each individual action model transition system is discrete, finite, and can be deterministic or not. The input on each transition is an action that can be carried out by the agent.

More information about the transition system can be found in the corresponding [wiki page](../../../wiki/Transition-System-Definition)

#### Plugins
A plugin system allows for integrating more feature to the planner node (notably used by the Human-In-the-Loop mixed initiative controller). Details on the plugin can be found on the [wiki page](../../../wiki/Planner-Plugin)

