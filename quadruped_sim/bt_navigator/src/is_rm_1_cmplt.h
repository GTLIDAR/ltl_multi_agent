#pragma once
#include <iostream>
#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class IsRm1Cmplt : public BT::ConditionNode
{
    public:
        IsRm1Cmplt( const std::string& name,
                    const BT::NodeConfiguration& config) :
            BT::ConditionNode(name, config) { }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<bool>("get_rm_1_cmplt") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<bool> msg = getInput<bool>("get_rm_1_cmplt");

            if(!msg) {
                throw BT::RuntimeError("missing required input [get_rm_1_cmplt]: ", msg.error() );
            }

            if(msg.value() == 1){
                std::cout << "rm1 spray succeeded" << std::endl;
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "rm1 spray failed" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        bool _aborted;
};