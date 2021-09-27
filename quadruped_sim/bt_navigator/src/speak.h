#pragma once

#include <behaviortree_cpp_v3/action_node.h>

class Speak : public BT::AsyncActionNode
{
    public:
        Speak(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("get_message") };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<std::string> msg = getInput<std::string>("get_message");
            // Check if optional is valid. If not, throw its error
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg.error() );
            }

            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
};
