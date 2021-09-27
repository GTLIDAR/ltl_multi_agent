//
// Created by ziyi on 6/23/21.
//

#ifndef QUADRUPED_CTRL_GAZEBO_INTERFACE_H
#define QUADRUPED_CTRL_GAZEBO_INTERFACE_H

#include "ros/ros.h"
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body.h"
#include <RobotRunner.h>
#include <RobotController.h>
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "custom_cmd_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include "quadruped_ctrl/QuadrupedCmd.h"

class Gazebo_interface {
public:
    Gazebo_interface(RobotController *robot_ctrl, std::string rname) : statusTask(&taskManager, 0.5f){
        _controller = robot_ctrl;
        _userControlParameters = robot_ctrl->getUserControlParameters();
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &Gazebo_interface::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &Gazebo_interface::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &Gazebo_interface::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &Gazebo_interface::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &Gazebo_interface::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &Gazebo_interface::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &Gazebo_interface::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &Gazebo_interface::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &Gazebo_interface::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &Gazebo_interface::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &Gazebo_interface::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &Gazebo_interface::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &Gazebo_interface::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &Gazebo_interface::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &Gazebo_interface::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &Gazebo_interface::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &Gazebo_interface::RLcalfCallback, this);
        joysticker_sub = nm.subscribe("/cmd_vel", 1, &Gazebo_interface::JoyStickerCallback, this);

    };
    ~Gazebo_interface() = default;
    void run();

    void imuCallback(const sensor_msgs::Imu & msg);
    void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RLfootCallback(const geometry_msgs::WrenchStamped& msg);

    bool ControlModeCallback(quadruped_ctrl::QuadrupedCmdRequest &req,
                             quadruped_ctrl::QuadrupedCmdResponse &res);

    void JoyStickerCallback(const geometry_msgs::Twist& msg);

private:
    PeriodicTaskManager taskManager;
    PrintTaskStatus statusTask;
    GamepadCommand _gamepadCommand;
    HighCmdCustom _highlevelCommand;
    VisualizationData _visualizationData;
    CheetahVisualization _mainCheetahVisualization;
    VectorNavData _vectorNavData;
    UNITREE_LEGGED_SDK::LowCmd _lowCmd{};
    UNITREE_LEGGED_SDK::LowState _lowState{};

    bool _firstRun = true;
    RobotRunner* _robotRunner = nullptr;
    RobotControlParameters _robotParams;
    u64 _iterations = 0;
    RobotController* _controller = nullptr;
    ControlParameters* _userControlParameters = nullptr;

    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, joysticker_sub;
    std::string robot_name;

};


#endif //QUADRUPED_CTRL_GAZEBO_INTERFACE_H
