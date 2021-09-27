//
// Created by ziyi on 7/21/21.
//

// file: listener.cpp
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>

#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <nav_msgs/Odometry.h>
#include "locomotion_status_lcmt.hpp"
#include "quadruped_ctrl/locomotion_status.h"

class Handler {
public:
    Handler(){loco_status_pub_ = nh_.advertise<quadruped_ctrl::locomotion_status>("/locomotion_status",1, true);}
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const locomotion_status_lcmt *msg)
    {
        quadruped_ctrl::locomotion_status status;
        status.operating_mode = msg->operating_mode;
        status.current_fsm = msg->current_fsm;
        loco_status_pub_.publish(status);
    }


private:
    ros::NodeHandle nh_;


    ros::NodeHandle* rosnode;
    ros::Publisher loco_status_pub_;

};

int main(int argc, char **argv)
{
    lcm::LCM lcm;
    ros::init(argc,argv,"loco_status_lcm2ros",ros::init_options::NoSigintHandler);
    if (!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("locomotion_status", &Handler::handleMessage, &handlerObject);

    while (0 == lcm.handle()) {
        // Do nothing
    }

    return 0;
}
