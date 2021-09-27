#pragma once

#include <behaviortree_cpp_v3/action_node.h>

class Spray : public BT::AsyncActionNode
{
    public:
        Spray(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            _aborted = false;
        }

        static BT::PortsList providedPorts()
        {
            return{                 
                    BT::InputPort<int>("get_curr_loc"),
                    BT::InputPort<bool>("get_rm_1_cmplt"),
                    BT::InputPort<bool>("get_rm_2_cmplt"),
                    BT::OutputPort<bool>("set_rm_1_cmplt"),
                    BT::OutputPort<bool>("set_rm_2_cmplt")
                };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<int> msg_curr_loc = getInput<int>("get_curr_loc");
            BT::Optional<bool> msg_rm_1_cmplt = getInput<bool>("get_rm_1_cmplt");
            BT::Optional<bool> msg_rm_2_cmplt = getInput<bool>("get_rm_2_cmplt");

            if(!msg_curr_loc) {
                throw BT::RuntimeError("missing required input [get_curr_loc]: ", msg_curr_loc.error() );
            }
            if(!msg_rm_1_cmplt) {
                throw BT::RuntimeError("missing required input [get_rm_1_cmplt]: ", msg_rm_1_cmplt.error() );
            }            
            if(!msg_rm_2_cmplt) {
                throw BT::RuntimeError("missing required input [get_rm_2_cmplt]: ", msg_rm_2_cmplt.error() );
            }

            switch(msg_curr_loc.value()) {
                case 1:
                    setOutput("set_rm_1_cmplt", true);
                    setOutput("set_rm_2_cmplt", msg_rm_2_cmplt.value());
                    std::cout << "Robot sprayed room 1" << std::endl;
                    break;
                case 2: 
                    setOutput("set_rm_1_cmplt", msg_rm_1_cmplt.value());
                    setOutput("set_rm_2_cmplt", true);
                    std::cout << "Robot sprayed room 2" << std::endl;
                    break;
                case 3: 
                    setOutput("set_rm_1_cmplt", msg_rm_1_cmplt.value());
                    setOutput("set_rm_2_cmplt", msg_rm_2_cmplt.value());
                    std::cout << "Robot is at home. There's nothing to spray" << std::endl;
                    break;
            }

            return BT::NodeStatus::SUCCESS;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
};