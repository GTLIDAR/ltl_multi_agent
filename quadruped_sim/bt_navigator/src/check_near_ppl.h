#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class CheckNearPpl : public BT::AsyncActionNode
{
    public:
        CheckNearPpl( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::AsyncActionNode(name, config) {  }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<bool>("set_near_ppl") };
        }

        virtual BT::NodeStatus tick() override
        {
            //Use sensor to detect nearby people
            setOutput("set_near_ppl", false);
            
            //TODO: If a person is detected, set true and return failure

            return BT::NodeStatus::SUCCESS;

        }
};
