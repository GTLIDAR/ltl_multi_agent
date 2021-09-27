#include "movebase_client.h"
#include "chk_low_battery.h"
#include "always_running.h"
#include "is_rm_cmplt.h"
#include "interrupt_event.h"
#include "send_cmd_vel.h"
// #include "slugs.h"
#include "speak.h"
#include "spray.h"

#ifdef SUPPORT_OPENVINO
  #include "openvino_event.h"
#endif
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// #include <nlohmann/json.hpp>
#include <iostream>

using namespace BT;
// using json = nlohmann::json;

int main(int argc, char **argv) {


  // // Import Slugs output json file into variable root
  // std::cout << "reading json" << std::endl;
  // std::string file_name = "/home/yuki/catkin_ws/src/BT_ros1/resource/quadruped.json";
  // std::ifstream myfile(file_name);
  // std::string line;
  // if(myfile.is_open())
  // {
  //   json root_;
  //   myfile >> root_;

  //   myfile.close();
  //   std::cout << "reading json config file completed." << std::endl;

  // } else {
  //   std::cout << "Could not find file name" << std::endl;
  // }
  
  ros::init(argc, argv, "test_bt");
  ros::NodeHandle nh("~");
  std::string xml_filename;

  //This file directory doesn't exist but it still finds the correct one...
  nh.param<std::string>("file", xml_filename, "/home/ros/catkin_kk_ws/src/BT_roskjkj1/BT_sample/cfg/bt_test.xml");
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  std::cout << "beginning factory" << std::endl;

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<SendCommandVel>("SendCommandVel");
  factory.registerSimpleCondition("CheckBattery", CheckBattery, {BT::InputPort<int>("wait_tick")});
  factory.registerNodeType<AlwaysRunning>("AlwaysRunning");
  factory.registerNodeType<InterruptEvent>("InterruptEvent");

  // Adding my custom node
  factory.registerNodeType<Speak>("Speak");
  factory.registerNodeType<Spray>("Spray");
  factory.registerNodeType<IsRmCmplt>("IsRmCmplt"); //condition
  // factory.registerNodeType<Slugs>("Slugs");


#ifdef SUPPORT_OPENVINO
  factory.registerNodeType<OpenVINOEvent>("OpenVINOEvent");
#endif

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {


    status = tree.rootNode()->executeTick();

    //First, attempt running two different trees at a different time
    //Second, see if you can access blackboard variables from here
    // Third, see if the variables can be changed here
    //All bb varoables are connected using ports???????? where do you put the key in for setoutput or getinput?

/*
yuki@yuki-Lenovo:~/catkin_ws/src/neuronbot2/neuronbot2_nav/maps$ roslaunch neuronbot2_gazebo neuronbot2_world.launch world_model:=house_world.model
yuki@yuki-Lenovo:~/catkin_ws$ roslaunch neuronbot2_nav bringup.launch open_rviz:=true local_planner:=dwa map:=$HOME/catkin_ws/src/neuronbot2/neuronbot2_nav/maps/house.yaml
yuki@yuki-Lenovo:~/catkin_ws$ roslaunch bt_sample bt_sample.launch
yuki@yuki-Lenovo:~/catkin_ws$ roslaunch bt_sample bt_basic.launch


roslaunch neuronbot2_nav bringup.launch open_rviz:=true local_planner:=dwa map:=$HOME/catkin_ws/src/neuronbot2/neuronbot2_nav/maps/mrdc_map.yaml
roslaunch neuronbot2_gazebo neuronbot2_world.launch world_model:=mrdc_world.model
roslaunch bt_sample bt_basic.launch



*/


    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
