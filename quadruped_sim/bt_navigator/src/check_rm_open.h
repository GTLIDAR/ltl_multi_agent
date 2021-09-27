#pragma once

#include <behaviortree_cpp_v3/action_node.h>

class CheckRmOpen : public BT::AsyncActionNode
{
    public:
        CheckRmOpen(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<bool>("set_is_door_open") };
        }

        virtual BT::NodeStatus tick() override
        {
            setOutput("set_is_door_open", true);

            //Force a failure when door is not open. 
            //Returning failure will prompt a different tree in SLUGS 
            //TODO: This is for the next implementation version


            std::cout << "Robot needs to scan" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
};