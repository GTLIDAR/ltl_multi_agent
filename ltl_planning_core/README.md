# ltl_automaton_core

## Overview
This metapackage provides an implementation of a LTL (Linear Temporal Logic) planner based on automaton graph. More information can be found on the [wiki](../../wiki) or in each package documentation.

### Publication
R. Baran, X. Tan, P. Varnai, P. Yu, S. Ahlberg, M. Guo, W. Shaw Cortez, and D. V. Dimarogonas, "A ROS Package for Human-In-the-Loop Planning and Control under Linear Temporal Logic Tasks", About to be presented at the IEEE 17th International Conference on Automation Science and Engineering (CASE), 2021. To cite this work:
```
@inproceedings{baran2021rosltl,
  title={A ROS Package for Human-In-the-Loop Planning and Control under Linear Temporal Logic Tasks},
  author={Robin Baran, Xiao Tan, Peter Varnai, Pian Yu, Sofie Ahlberg, Meng Guo, Wenceslao Shaw Cortez and Dimos V. Dimarogonas},
  booktitle={IEEE 17th International Conference on Automation Science and Engineering (CASE)},
  year={2021}
}
```

## Installation

### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org)(duh), package tested on Kinetic, Melodic and Morenia distributions

- [LTL2BA](https://github.com/KTH-DHSG/ros_ltl2ba). ROS package wrapping for the LTL2BA software by Dennis Oddoux and Paul Gastin.
    - Clone the repository from Github in your catkin workspace:
    ```
    cd catkin_ws/src
    git clone https://github.com/KTH-DHSG/ros_ltl2ba
    ```
    - Build your workspace with your prefered tool:
    `cd ..`
    `catkin_make` or `catkin build`

- [PLY (Python Lex-Yacc)](http://www.dabeaz.com/ply/)
	- For Python2 (ROS Kinetic & Melodic):
	`pip install ply`
	- For Python3 (ROS Morenia):
	`pip3 install ply`

- [NetworkX](https://networkx.org/). Software for complex networks analysis in Python.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install networkx`
	- For Python3 (ROS Morenia):
	`pip3 install networkx`

- [PyYAML](https://pyyaml.org/). Should be integrated with ROS but it's better to check if version is up-to-date.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install pyyaml`
	- For Python3 (ROS Morenia):
	`pip3 install pyyaml`


### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/KTH-SML/ltl_automaton_core.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage

This package is not meant to be used on its own. It provides an LTL planner node that can be interfaced with an external agent package. Nevertheless an example launch file can be run using the following command:

```
roslaunch ltl_automaton_planner ltl_planner_example.launch

```

It will launch a planner using the formula provided by the config file *ltl_automaton_planner/config/example_ltl_formula* and using the transition system specified in *ltl_automaton_planner/config/example_ts*.

## Packages
This metapackage is composed of the following packages.

- **[ltl_automaton_planner](/ltl_automaton_planner)**: Provides the LTL planner node. The node uses a transition system and a LTL formula to generate a plan and action sequence, and update them according to agent state.

- **[ltl_automaton_msgs](/ltl_automaton_msgs)**: A message definition packages for the LTL automaton packages.

- **[ltl_automaton_std_transition_systems](/ltl_automaton_std_transition_systems)**: A set of state monitors for standard transition systems (2D regions, ...).

- **[ltl_automaton_hil_mic](/ltl_automaton_hil_mic)**: An implementation of a Human-In-the-Loop mix initiative controller for agents using the LTL planner.
