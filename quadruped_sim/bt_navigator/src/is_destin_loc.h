#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/condition_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class IsDestinLoc : public BT::ConditionNode
{
    public:
        IsDestinLoc( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {  }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<int>("const_loc_num"), 
                    BT::InputPort<int>("get_destination") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<int> msg_const_loc_num = getInput<int>("const_loc_num");
            BT::Optional<int> msg_destination = getInput<int>("get_destination");

            if(!msg_const_loc_num) {
                throw BT::RuntimeError("missing required input [const_loc_num]: ", msg_const_loc_num.error() );
            }
            if(!msg_destination) {
                throw BT::RuntimeError("missing required input [get_destination]: ", msg_destination.error() );
            }

            if(msg_const_loc_num.value() == msg_destination.value()){
                std::cout << "action " << msg_const_loc_num.value() << " chosen" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }
};
