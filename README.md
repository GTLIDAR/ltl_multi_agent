# Reactive task allocation and planning of a heterogeneous multi-robot system
## Overview
This repo contains all the packages for multi-agent task allocation and planning based on Linear Temporal Logic (LTL). 
Each component is maintained individually.

## Packages
This metapackage is composed of the following packages.

- **[ltl_planning_core](/ltl_planning_core)**: A metapackage in charge of multi-robot task allocation and planning.

- **[ltl_automation_a1](/ltl_automation_a1)**: Behavior tree nodes for online task execution. Quadruped and wheeled robots are both included.

- **[quadruped_sim](/quadruped_sim)**: Essential components for quadruped including Gazebo simulation, navigation, and low-level control. 

- **[motion_capture_simulator](/motion_capture_simulator)**:

- **[ubtna_gazebo_pkg](/ubtna_gazebo_pkg)**: Essential components for wheeled robots from UBTECH including Gazebo simulation, navigation

- **[Groot](/Groot) (optional)**: GUI for behavior tree
 

## Installation
Before building the whole workspace, please follow the instructions in each subpackage above to install required dependencies.

All packages shall be located in the same catkin workspace. To build the packages, follow:
```
$ cd catkin_ws/src
$ git clone https://github.com/GTLIDAR/ltl_multi_agent
$ cd ..
$ catkin build
$ source devel/setup.bash
```

## Usage
Run a case study that involves a quadruped A1, a delivery robot DR and a walk training robot Wassi (DR and Wassi models
are proprietary and presented by a simple model):

In terminal 1, start gazebo simulation and BT nodes for each robot. Try rerunning the launch file if models
are not spawned correctly (to be fixed).
```
$ roslaunch ltl_automaton_planner study_2.launch
```

In terminal 2, start quadruped controller:
```
$ rosrun quadruped_ctrl a1_servo /cmd_vel:=/a1_gazebo/cmd_vel
```

In terminal 3, start LTL planner if quadruped stands up and laser scans are received in Rviz:
```
$ rosrun ltl_automaton_planner robot_planner_node_exp
```

## Reference Citation
To cite this work:
```
@article{zhou2021reactive,
  title={Reactive Task Allocation and Planning of A Heterogeneous Multi-Robot System},
  author={Zhou, Ziyi and Lee, Dong Jae and Yoshinaga, Yuki and Guo, Dejun and Zhao, Ye},
  journal={arXiv e-prints},
  pages={arXiv--2110},
  year={2021}
}
```

