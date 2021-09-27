//
// Created by ziyi on 6/23/21.
//

#include "Gazebo_interface.h"
#include "convert.h"

using namespace std;
using namespace unitree_model;

void Gazebo_interface::run()
{
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state
    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    // ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
    ros::ServiceServer service = nm.advertiseService("/ControlMode", &Gazebo_interface::ControlModeCallback, this);

    unitree_model::lowState = ToRos(_lowState);
//    motion_init();

    ROS_INFO("[Gazebo Bridge] Loading parameters from file...\n");
    std::string package_name = "quadruped_ctrl";
    std::string robot_yaml = ros::package::getPath(package_name).append("/config/a1-defaults.yaml");
    try {
        _robotParams.initializeFromYamlFile(robot_yaml);
    } catch(std::exception& e) {
        ROS_ERROR("Failed to initialize robot parameters from yaml file: %s\n", e.what());
        exit(1);
    }

    if(!_robotParams.isFullyInitialized()) {
        ROS_ERROR("Failed to initialize all robot parameters\n");
        exit(1);
    }

    ROS_INFO("Loaded robot parameters\n");

    if(_userControlParameters) {
        std::string user_yaml = ros::package::getPath(package_name).append("/config/a1-mit-ctrl-user-parameters.yaml");
        try {
            _userControlParameters->initializeFromYamlFile(user_yaml);
        } catch(std::exception& e) {
            ROS_ERROR("Failed to initialize user parameters from yaml file: %s\n", e.what());
            exit(1);
        }

        if(!_userControlParameters->isFullyInitialized()) {
            ROS_ERROR("Failed to initialize all user parameters\n");
            exit(1);
        }

        ROS_INFO("Loaded user parameters\n");
    } else {
        ROS_INFO("Did not load user parameters because there aren't any\n");
    }



    printf("[Hardware Bridge] Got all parameters, starting up!\n");

    _robotRunner =
            new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

    _robotRunner->driverCommand = &_gamepadCommand;
    _robotRunner->HighlevelCmd = &_highlevelCommand;
    _robotRunner->vectorNavData = &_vectorNavData;
    _robotRunner->a1Data = &_lowState;
    _robotRunner->a1Command = &_lowCmd;
    _robotRunner->robotType = RobotType::A1;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData = &_visualizationData;
    _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

    _firstRun = false;

    // init control thread

    statusTask.start();

    // robot controller start; lcm msgs are published in RobotRunner
    _robotRunner->start();

    //Declare some variables
    double imu_data[10];
    double leg_data[24]; // [q, qdot]
    double effort_arr[12]; //[tau]

//    RobotController* rbt_ctrl;
//    rbt_ctrl->runController();

    // Sending the control to gazebo and publish to rviz
    while (ros::ok()){
        unitree_model::lowState = ToRos(_lowState);
        lowState_pub.publish(unitree_model::lowState);

        //Put gait controller::torque calculator here
        for(int i = 0; i < 10; i++){
            if(i < 3){
                imu_data[i] = unitree_model::lowState.imu.accelerometer[i];
            }
            else if(i < 7){
                imu_data[i] = unitree_model::lowState.imu.quaternion[i-3];
            }
            else {
                imu_data[i] = unitree_model::lowState.imu.gyroscope[i-7];
            }
//            imu_data[i] = 0.0;
        }
//        std::cout << "imu: " << imu_data[0] << " " << imu_data[1] << " " << imu_data[2] << " "
//                  << imu_data[3] << " " << imu_data[4] << " " << imu_data[5] << " "
//                  << imu_data[6] << " " << imu_data[7] << " " << imu_data[8] << " "
//                  << imu_data[9] << std::endl;


        for(int i = 0; i < 12; i++){
            leg_data[i] = lowState.motorState[i].q;
            leg_data[i+12] = lowState.motorState[i].dq;
        }


        for(int i = 0; i < 12; i++){
            unitree_model::lowCmd.motorCmd[i].tau = effort_arr[i];
        }

        unitree_model::lowCmd = ToRos(_lowCmd);
//        for(int i=0; i<4; i++){
//            unitree_model::lowCmd.motorCmd[i*3+0].mode = 0x0A;
//            unitree_model::lowCmd.motorCmd[i*3+0].Kp = 0;
//            unitree_model::lowCmd.motorCmd[i*3+0].dq = 0;
//            unitree_model::lowCmd.motorCmd[i*3+0].Kd = 0;
////            lowCmd.motorCmd[i*3+0].tau = effort_arr[i*3+0];
//            unitree_model::lowCmd.motorCmd[i*3+0].tau = 0.0;
//            unitree_model::lowCmd.motorCmd[i*3+1].mode = 0x0A;
//            unitree_model::lowCmd.motorCmd[i*3+1].Kp = 0;
//            unitree_model::lowCmd.motorCmd[i*3+1].dq = 0;
//            unitree_model::lowCmd.motorCmd[i*3+1].Kd = 0;
////            lowCmd.motorCmd[i*3+1].tau = effort_arr[i*3+1];
//            unitree_model::lowCmd.motorCmd[i*3+1].tau = 0.0;
//            unitree_model::lowCmd.motorCmd[i*3+2].mode = 0x0A;
//            unitree_model::lowCmd.motorCmd[i*3+2].Kp = 0;
//            unitree_model::lowCmd.motorCmd[i*3+2].dq = 0;
//            unitree_model::lowCmd.motorCmd[i*3+2].Kd = 0;
////            lowCmd.motorCmd[i*3+2].tau = effort_arr[i*3+2];
//            unitree_model::lowCmd.motorCmd[i*3+2].tau = 0.0;
//        }

        sendServoCmd();

    }
}

void Gazebo_interface::imuCallback(const sensor_msgs::Imu & msg)
{
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;

    _vectorNavData.accelerometer = Eigen::Vector3f(_lowState.imu.accelerometer);
    // TODO: double check the order for quat of _vectorNavData
    _vectorNavData.quat[0] = _lowState.imu.quaternion[1];
    _vectorNavData.quat[1] = _lowState.imu.quaternion[2];
    _vectorNavData.quat[2] = _lowState.imu.quaternion[3];
    _vectorNavData.quat[3] = _lowState.imu.quaternion[0];
    _vectorNavData.gyro = Eigen::Vector3f(_lowState.imu.gyroscope);

//        cout << "acc: " << lowState.imu.accelerometer[0] << " " << lowState.imu.accelerometer[1] << " "
//                       << lowState.imu.accelerometer[2] << endl;

}

void Gazebo_interface::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void Gazebo_interface::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void Gazebo_interface::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void Gazebo_interface::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void Gazebo_interface::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void Gazebo_interface::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void Gazebo_interface::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void Gazebo_interface::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void Gazebo_interface::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void Gazebo_interface::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void Gazebo_interface::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void Gazebo_interface::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

void Gazebo_interface::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    unitree_model::lowState.eeForce[0].x = msg.wrench.force.x;
    unitree_model::lowState.eeForce[0].y = msg.wrench.force.y;
    unitree_model::lowState.eeForce[0].z = msg.wrench.force.z;
    _lowState.footForce[0] = msg.wrench.force.z;
}

void Gazebo_interface::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    unitree_model::lowState.eeForce[1].x = msg.wrench.force.x;
    unitree_model::lowState.eeForce[1].y = msg.wrench.force.y;
    unitree_model::lowState.eeForce[1].z = msg.wrench.force.z;
    _lowState.footForce[1] = msg.wrench.force.z;
}

void Gazebo_interface::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    unitree_model::lowState.eeForce[2].x = msg.wrench.force.x;
    unitree_model::lowState.eeForce[2].y = msg.wrench.force.y;
    unitree_model::lowState.eeForce[2].z = msg.wrench.force.z;
    _lowState.footForce[2] = msg.wrench.force.z;
}

void Gazebo_interface::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    unitree_model::lowState.eeForce[3].x = msg.wrench.force.x;
    unitree_model::lowState.eeForce[3].y = msg.wrench.force.y;
    unitree_model::lowState.eeForce[3].z = msg.wrench.force.z;
    _lowState.footForce[3] = msg.wrench.force.z;
}

bool Gazebo_interface::ControlModeCallback(quadruped_ctrl::QuadrupedCmdRequest &req,
                                           quadruped_ctrl::QuadrupedCmdResponse &res) {
    _highlevelCommand.mode = req.cmd;
    res.result = 0;
    res.description = "Get new control mode";
    ROS_INFO("Control mode switch to: [%hd]", req.cmd);
    return true;
}

void Gazebo_interface::JoyStickerCallback(const geometry_msgs::Twist &msg) {
    _highlevelCommand.forwardSpeed = msg.linear.x;
    _highlevelCommand.sideSpeed = msg.linear.y;
    _highlevelCommand.rotateSpeed = msg.angular.z;
}
