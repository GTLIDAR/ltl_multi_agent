//
// Created by ziyi on 6/23/21.
//

#include <ros/ros.h>
#include "Gazebo_interface.h"
#include <MIT_Controller.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "a1_gazebo_control");
    Gazebo_interface interface(new MIT_Controller, "a1");
    interface.run();
    ros::spin();
    return 0;
}