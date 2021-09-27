#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class IsRm2Cmplt : public BT::ConditionNode
{
    public:
        IsRm2Cmplt( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {  }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<bool>("get_rm_2_cmplt") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<bool> msg = getInput<bool>("get_rm_2_cmplt");

            if(!msg) {
                throw BT::RuntimeError("missing required input [get_rm_2_cmplt]: ", msg.error() );
            }

            if(msg.value() == 1){
                std::cout << "rm2 spray succeeded" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "rm2 spray failed" << std::endl;
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

