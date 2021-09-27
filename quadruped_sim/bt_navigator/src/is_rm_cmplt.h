#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class IsRmCmplt : public BT::ConditionNode
{
    public:
        IsRmCmplt( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {  }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<int>("get_rm_num"), 
                    BT::InputPort<bool>("get_rm_cmplt") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<int> msg_rm_num = getInput<int>("get_rm_num");
            BT::Optional<bool> msg_rm_cmplt = getInput<bool>("get_rm_cmplt");

            if(!msg_rm_num) {
                throw BT::RuntimeError("missing required input [get_rm_num]: ", msg_rm_num.error() );
            }
            if(!msg_rm_cmplt) {
                throw BT::RuntimeError("missing required input [msg_rm_cmplt]: ", msg_rm_cmplt.error() );
            }

            if(msg_rm_cmplt.value() == true){
                std::cout << "rm " << msg_rm_num.value() << " spray succeeded" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "rm" << msg_rm_num.value()  << " spray failed" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        bool _aborted;
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

