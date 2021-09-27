//
// Created by ziyi on 6/23/21.
//

#ifndef QUADRUPED_CTRL_CONVERT_H
#define QUADRUPED_CTRL_CONVERT_H
/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>

#include "aliengo_sdk/aliengo_sdk.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

unitree_legged_msgs::IMU ToRos(UNITREE_LEGGED_SDK::IMU& lcm)
{
    unitree_legged_msgs::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.accelerometer[0];
    ros.accelerometer[1] = lcm.accelerometer[1];
    ros.accelerometer[2] = lcm.accelerometer[2];
    // ros.rpy[0] = lcm.rpy[0];
    // ros.rpy[1] = lcm.rpy[1];
    // ros.rpy[2] = lcm.rpy[2];
    ros.temperature = lcm.temperature;
    return ros;
}

UNITREE_LEGGED_SDK::IMU ToLcm(unitree_legged_msgs::IMU& ros)
{
    UNITREE_LEGGED_SDK::IMU lcm;
    lcm.quaternion[0] = ros.quaternion[0];
    lcm.quaternion[1] = ros.quaternion[1];
    lcm.quaternion[2] = ros.quaternion[2];
    lcm.quaternion[3] = ros.quaternion[3];
    lcm.gyroscope[0] = ros.gyroscope[0];
    lcm.gyroscope[1] = ros.gyroscope[1];
    lcm.gyroscope[2] = ros.gyroscope[2];
    lcm.accelerometer[0] = ros.accelerometer[0];
    lcm.accelerometer[1] = ros.accelerometer[1];
    lcm.accelerometer[2] = ros.accelerometer[2];
    // lcm.rpy[0] = lcm.rpy[0];
    // ros.rpy[1] = lcm.rpy[1];
    // ros.rpy[2] = lcm.rpy[2];
    lcm.temperature = ros.temperature;
    return lcm;
}

unitree_legged_msgs::MotorState ToRos(UNITREE_LEGGED_SDK::MotorState& lcm)
{
    unitree_legged_msgs::MotorState ros;
    ros.mode = lcm.mode;
    ros.q = lcm.q;
    ros.dq = lcm.dq;
    ros.ddq = lcm.ddq;
    ros.tauEst = lcm.tauEst;
    ros.q_raw = lcm.q_raw;
    ros.dq_raw = lcm.dq_raw;
    ros.ddq_raw = lcm.ddq_raw;
    ros.temperature = lcm.temperature;
    ros.reserve[0] = lcm.reserve[0];
    ros.reserve[1] = lcm.reserve[1];
    return ros;
}

UNITREE_LEGGED_SDK::MotorState ToLcm(unitree_legged_msgs::MotorState& ros)
{
    UNITREE_LEGGED_SDK::MotorState lcm;
    lcm.mode = ros.mode;
    lcm.q = ros.q;
    lcm.dq = ros.dq;
    lcm.ddq = ros.ddq;
    lcm.tauEst = ros.tauEst;
    lcm.q_raw = ros.q_raw;
    lcm.dq_raw = ros.dq_raw;
    lcm.ddq_raw = ros.ddq_raw;
    lcm.temperature = ros.temperature;
    lcm.reserve[0] = ros.reserve[0];
    lcm.reserve[1] = ros.reserve[1];
    return lcm;
}

unitree_legged_msgs::MotorCmd ToRos(UNITREE_LEGGED_SDK::MotorCmd& lcm)
{
    unitree_legged_msgs::MotorCmd ros;
    ros.mode = lcm.mode;
    ros.q = lcm.q;
    ros.dq = lcm.dq;
    ros.tau = lcm.tau;
    ros.Kp = lcm.Kp;
    ros.Kd = lcm.Kd;
    ros.reserve[0] = lcm.reserve[0];
    ros.reserve[1] = lcm.reserve[1];
    ros.reserve[2] = lcm.reserve[2];
    return ros;
}

UNITREE_LEGGED_SDK::MotorCmd ToLcm(unitree_legged_msgs::MotorCmd& ros)
{
    UNITREE_LEGGED_SDK::MotorCmd lcm;
    lcm.mode = ros.mode;
    lcm.q = ros.q;
    lcm.dq = ros.dq;
    lcm.tau = ros.tau;
    lcm.Kp = ros.Kp;
    lcm.Kd = ros.Kd;
    lcm.reserve[0] = ros.reserve[0];
    lcm.reserve[1] = ros.reserve[1];
    lcm.reserve[2] = ros.reserve[2];
    return lcm;
}

unitree_legged_msgs::LowState ToRos(UNITREE_LEGGED_SDK::LowState& lcm)
{
    unitree_legged_msgs::LowState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.imu = ToRos(lcm.imu);
    for(int i = 0; i<20; i++){
        ros.motorState[i] = ToRos(lcm.motorState[i]);
    }
    for(int i = 0; i<4; i++){
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::LowState& ToLcm(unitree_legged_msgs::LowState& ros)
{
    UNITREE_LEGGED_SDK::LowState lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    lcm.imu = ToLcm(ros.imu);
    for(int i = 0; i<20; i++){
        lcm.motorState[i] = ToLcm(ros.motorState[i]);
    }
    for(int i = 0; i<4; i++){
        lcm.footForce[i] = ros.footForce[i];
        lcm.footForceEst[i] = ros.footForceEst[i];
    }
    lcm.tick = ros.tick;
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}

unitree_legged_msgs::LowCmd ToRos(UNITREE_LEGGED_SDK::LowCmd& lcm)
{
    unitree_legged_msgs::LowCmd ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    for(int i = 0; i<20; i++){
        ros.motorCmd[i] = ToRos(lcm.motorCmd[i]);
    }
    for(int i = 0; i<4; i++){
        ros.led[i].r = lcm.led[i].r;
        ros.led[i].g = lcm.led[i].g;
        ros.led[i].b = lcm.led[i].b;
    }
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::LowCmd ToLcm(unitree_legged_msgs::LowCmd& ros)
{
    UNITREE_LEGGED_SDK::LowCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    for(int i = 0; i<20; i++){
        lcm.motorCmd[i] = ToLcm(ros.motorCmd[i]);
    }
    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}

unitree_legged_msgs::HighState ToRos(UNITREE_LEGGED_SDK::HighState& lcm)
{
    unitree_legged_msgs::HighState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.mode = lcm.mode;
    ros.imu = ToRos(lcm.imu);
    ros.forwardSpeed = lcm.forwardSpeed;
    ros.sideSpeed = lcm.sideSpeed;
    ros.rotateSpeed = lcm.rotateSpeed;
    ros.bodyHeight = lcm.bodyHeight;
    ros.updownSpeed = lcm.updownSpeed;
    ros.forwardPosition = lcm.forwardPosition;
    ros.sidePosition = lcm.sidePosition;
    for(int i = 0; i<4; i++){
        ros.footPosition2Body[i].x = lcm.footPosition2Body[i].x;
        ros.footPosition2Body[i].y = lcm.footPosition2Body[i].y;
        ros.footPosition2Body[i].z = lcm.footPosition2Body[i].z;
        ros.footSpeed2Body[i].x = lcm.footSpeed2Body[i].x;
        ros.footSpeed2Body[i].y = lcm.footSpeed2Body[i].y;
        ros.footSpeed2Body[i].z = lcm.footSpeed2Body[i].z;
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros)
{
    UNITREE_LEGGED_SDK::HighCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    lcm.mode = ros.mode;
    lcm.forwardSpeed = ros.forwardSpeed;
    lcm.sideSpeed = ros.sideSpeed;
    lcm.rotateSpeed = ros.rotateSpeed;
    lcm.bodyHeight = ros.bodyHeight;
    lcm.footRaiseHeight = ros.footRaiseHeight;
    lcm.yaw = ros.yaw;
    lcm.pitch = ros.pitch;
    lcm.roll = ros.roll;
    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
        lcm.AppRemote[i] = ros.AppRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}
#endif
