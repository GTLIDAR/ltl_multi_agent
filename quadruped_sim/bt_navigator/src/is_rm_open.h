#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class IsRmOpen : public BT::ConditionNode
{
    public:
        IsRmOpen( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {  }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<int>("get_rm_num"), 
                    BT::InputPort<bool>("get_rm_open") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<int> msg_rm_num = getInput<int>("get_rm_num");
            BT::Optional<bool> msg_rm_open = getInput<bool>("get_rm_open");

            if(!msg_rm_num) {
                throw BT::RuntimeError("missing required input [get_rm_num]: ", msg_rm_num.error() );
            }
            if(!msg_rm_open) {
                throw BT::RuntimeError("missing required input [msg_rm_open]: ", msg_rm_open.error() );
            }

            if(msg_rm_open.value() == true){
                std::cout << "rm " << msg_rm_num.value() << " open succeeded" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "rm" << msg_rm_num.value()  << " open failed" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
};






// class WaypointReached : public BT::ConditionNode
// {
//   public:
//   WaypointReached(const std::string& name,
//                   const NodeConfiguration& config) :
//     BT::ConditionNode(name, config) {}

//   static PortsList providedPorts()
//   {
//     return { InputPort<double>("threshold_distance") };
//   }

//   BT::NodeStatus tick() override {

//     Optional<double> msg = getInput<double>("threshold_distance");

//     if(msg) {
//       std::cout << msg.value() << std::endl;
//       return BT::NodeStatus::SUCCESS;
//     }
//     else{
//       std::cerr << "missing parameter [threshold_distance]" << std::endl;
//       return BT::NodeStatus::FAILURE;
//     }
//   }
// };

// int main(int argc, char** argv) 
// {
//   BehaviorTreeFactory factory;
//   factory.registerNodeType<WaypointReached>("WaypointReached");
//   return 0;
// }

