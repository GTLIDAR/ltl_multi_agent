#pragma once
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include "slugs_data_interface.h"

using json = nlohmann::json;

class CheckSlugs : public BT::AsyncActionNode
{
    public:
        CheckSlugs(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config) { }     
            //TODO: 
            // Simulate blockage
            // Step 1: Change BT tree and incorporate detection of door closed ......................(done)
            // Step 2: Create input ports for slugs.h
            // Step 3: Make sure the slugs data updates to the correct state
            // Step 4: Check if the robot changes to different room if current one is blocked
            // Step 5: Simulate
            // Let the robot speak the time

            //Note: 
            // Robot may not move at first since rooms are locked at time 30
            // Things to update
                //Robot battery
                //Room meeting locked
                //If robot is near people 


        // We want this method to be called ONCE and BEFORE the first tick()
        void init(json root)
        {
            root_ = root;
            _aborted = false;

            std::cout << "SANITY CHECK : this should be 0.0.1: " << root["slugs"] << std::endl;

            //Use the "new" keyword to ensure variable doesn't get deleted after exiting init()
            slugs_data_interface_ = new SlugsDataInterface(root);
        }

        static BT::PortsList providedPorts()
        {
            // BT::InputPort<bool>("get_time"),
            // BT::InputPort<bool>("get_battery_level"),

            return{ 
                BT::InputPort<int>("get_curr_location"),
                BT::InputPort<bool>("get_near_ppl"),
                BT::InputPort<bool>("get_rm_1_open"),   
                BT::InputPort<bool>("get_rm_2_open"),
                BT::OutputPort<int>("set_action"),
                BT::OutputPort<int>("set_destination")
             };
        }

        virtual BT::NodeStatus tick() override
        {
            BT::Optional<int> msg_curr_location = getInput<int>("get_curr_location");
            BT::Optional<int> msg_get_near_ppl = getInput<int>("get_near_ppl");
            BT::Optional<int> msg_get_rm_1_open = getInput<int>("get_rm_1_open");
            BT::Optional<int> msg_get_rm_2_open = getInput<int>("get_rm_2_open");

            // Check if optional is valid. If not, throw its error
            if (!msg_curr_location) {
                throw BT::RuntimeError("missing required input [msg_curr_location]: ", msg_curr_location.error() );
            }
            if (!msg_get_near_ppl){
                throw BT::RuntimeError("missing required input [msg_get_near_ppl]: ", msg_get_near_ppl.error() );
            }
            if (!msg_get_rm_1_open) {
                throw BT::RuntimeError("missing required input [msg_get_rm_1_open]: ", msg_get_rm_1_open.error() );
            }
            if (!msg_get_rm_2_open) {
                throw BT::RuntimeError("missing required input [msg_get_rm_2_open]: ", msg_get_rm_2_open.error() ); 
            }

            std::cout << "curr_loc: " << msg_curr_location.value() << std::endl;
            std::cout << "near_ppl: " << msg_get_near_ppl.value() << std::endl;
            std::cout << "rm_1_open: " << msg_get_rm_1_open.value() << std::endl;
            std::cout << "rm_2_open: " << msg_get_rm_2_open.value() << std::endl;

            //Currently, the battery level will remain 10 at all times.
            if(slugs_data_interface_->set_env_var(
                msg_get_rm_1_open.value(), msg_get_rm_2_open.value(), 
                9, msg_get_near_ppl.value()))
            {
                return BT::NodeStatus::FAILURE;
            }

            //Get the new state's action and location choice
            setOutput("set_action", slugs_data_interface_->get_action());
            setOutput("set_destination", slugs_data_interface_->get_loc_action());
            std::cout << "action: " << slugs_data_interface_->get_action() << std::endl;
            std::cout << "destination: " << slugs_data_interface_->get_loc_action() << std::endl;


            return BT::NodeStatus::SUCCESS;
        }
     
        virtual void halt() override
        {
            _aborted = true;
        }

    private:
        bool _aborted;
        SlugsDataInterface *slugs_data_interface_;
        json root_;
};
