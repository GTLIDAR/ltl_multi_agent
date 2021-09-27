# A1_ros for Gazebo interface

### A1 simulation in gazebo with MIT Controller
This repo is for A1 gazebo simulation.
### System requirements:
Ubuntu 18.04, ROS Mellodic (tested by Ziyi 23/06/2021) 

### Dependency:
* [unitree_ros](https://github.com/unitreerobotics/unitree_ros)

* [Cheetah-Software](https://github.gatech.edu/GeorgiaTechLIDARGroup/Cheetah-Software) (checkout to gazebo_supported branch)

ROS joy for gamepad control (TBD)

### Build
First add your Cheetah-Software path into bashrc:
```
export CHEETAH_SOFTWARE_PATH=/path/to/Cheetah-Software
```
Then build everything
```
cd {your workspace}/src
git clone https://github.gatech.edu/GeorgiaTechLIDARGroup/A1_ROS.git
cd ..
catkin_make
source devel/setup.bash
```

### Running:
run the gazebo simulator
```
roslaunch unitree_gazebo normal.launch rname:=a1
```
run the controller:  
```
rosrun quadruped_ctrl a1_servo
```

Switch the control mode:
```
rosservice call /Controlmode 0
```
Mode 0 for recovery stand; 1 for balance stand; 2 for locomotion.
