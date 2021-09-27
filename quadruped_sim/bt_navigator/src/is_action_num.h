#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/condition_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class IsActionNum : public BT::ConditionNode
{
    public:
        IsActionNum( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) {  }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<int>("const_action_num"), 
                    BT::InputPort<int>("get_action_num") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<int> msg_const_action_num = getInput<int>("const_action_num");
            BT::Optional<int> msg_action_num = getInput<int>("get_action_num");

            if(!msg_const_action_num) {
                throw BT::RuntimeError("missing required input [const_action_num]: ", msg_const_action_num.error() );
            }
            if(!msg_action_num) {
                throw BT::RuntimeError("missing required input [get_action_num]: ", msg_action_num.error() );
            }

            if(msg_const_action_num.value() == msg_action_num.value()){
                std::cout << "action " << msg_const_action_num.value() << " chosen" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }
};
