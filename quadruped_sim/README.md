# Introduction
This repo combines unitree's ROS package with MIT's minicheetah WBIC + Convex MPC for simulation using ROS navigation stack. This repo is built on top of the [unitree_ros](https://github.com/unitreerobotics/unitree_ros.git). The inspiration was from [CHAMP](https://github.com/chvmp/champ) quadruped github, yet this repo is an upgrade to incorporate improved minicheetah controller into the Unitree A1's hardware. This repo uses [iris_lama](https://github.com/iris-ua/iris_lama) & [iris_lama_ros](https://github.com/iris-ua/iris_lama_ros) for SLAM.

# System Requirements
This repo has been built and tested on both Ubuntu 20.04 / 18.04 with ROS Noetic / Melodic. Please have suitable ROS1 version fully installed on your system along with g++/gcc compiler version 7.5.0. For having multiple versions of g++/gcc compiler on your system, please refer to [this link](https://github.com/vlfeat/matconvnet/issues/967#issuecomment-304977445) for tips. 

# Components
This repo ONLY contains the quadruped_sim part of the below tree, which is a gazebo & ROS interface to the minicheetah controller and the navigation stack setup. Your home directory should contain the following.

* your_ws
    * src
        * [quadruped_sim](https://github.gatech.edu/GeorgiaTechLIDARGroup/quadruped_sim) - THIS REPO
        * [motion_capture_simulator](https://github.com/KTH-SML/motion_capture_simulator) - MUST (for localization)
        * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (optional)
        * [Groot](https://github.com/BehaviorTree/Groot) (optional)

* Cheetah-Software <gazebo-supported>
    * build
    * ... etc
* LCM
    * build
    * ... etc
* unitree_legged_sdk
    * build
    * ... etc
* aliengo_sdk
    * build
    * ... etc
* QT


# Step 1: Setting Up Dependencies At Home Directory
Before using this package, please make sure to have `gazebo supported` branch of [Cheetah-Software](https://github.gatech.edu/GeorgiaTechLIDARGroup/Cheetah-Software) built on your environment. You will need to download and install pyQt5, LCM, Eigen, and couple other dependencies for this step. Refer to the components structure diagram above to finish setting up the dependencies at your home directory. More details are provided at the link to Cheetah-Software. For more information on how the WBIC + Convec MPC works, please refer to [this paper](https://arxiv.org/pdf/1909.06586.pdf). 

# Step 2: Setting Up Your catkin_ws
Your catkin_ws will consist of this repo, iris_lama & iris_lama_ros repo for SLAM, and BehaviorTree.CPP & Groot for BT implementation. Please `$ git clone` appropriate repos & braches into your `catkin_ws/src`.

## Sub Packages Within Your quadruped_sim:
Robot descriptions: `a1_description`, `aliengo_description`, `laikago_description`

Robot and joints controller: `unitree_controller`, `minicheetah_controller`

Basic function: `unitree_legged_msgs`

Simulation related: `unitree_bringup`, `unitree_navigate`

Real robot control related: `unitree_legged_real`

Plugins: `ouster_gazebo_plugins`, `velodyne_gazebo_plugins`

Behavior-tree related: `bt_navigator`


# Step 3: Build
```
$ cd catkin_ws
$ catkin config --extend /opt/ros/noetic
$ catkin build
$ source ./devel/setup.bash
```
If the build fails due to `unitree_legged_msgs/LowCmd.h: No such file or directory` error, try to compile everything except `minicheetah_controller` package first, then paste the folder back into the `quadruped_sim` to recompile.

# Step 4: Quick Demo
On terminal 1, launch the robot into your gazebo world, initiate move_base, localization, and rviz. 
```
$ roslaunch unitree_bringup bringup.launch world_name:=mrdc_map world_format:=model
```

On terminal 2, trigger the a1's standup motion to start the robot. Without this step, the robot won't be able to move around.
```
$ rosrun quadruped_ctrl a1_servo
```
![Standup Example](https://github.gatech.edu/GeorgiaTechLIDARGroup/quadruped_sim/blob/master/figures/standup.gif)
![Navigation Example](https://github.gatech.edu/GeorgiaTechLIDARGroup/quadruped_sim/blob/master/figures/success.gif)

On terminal 3, launch the localizaition, move_base, and rviz for navigation. You can choose to use mocap result for odom (groud truth) or use amcl for laser localization. Currently, using the mocap's localization is recommended due to lack of tuning on amcl.
```
$ roslaunch unitree_navigate navigate_mocap.launch 
$ roslaunch unitree_navigate navigate_amcl.launch
```

On termianl 4, launch the BT example to move between different waypoints
$ roslaunch bt_navigator bt_basic.launch 
```


