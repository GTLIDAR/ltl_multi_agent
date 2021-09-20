//
// Created by ziyi on 7/9/21.
//

#ifndef LTL_AUTOMATION_A1_NAVIGATION_NODE_H
#define LTL_AUTOMATION_A1_NAVIGATION_NODE_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include <string>
#include "ros/ros.h"
#include "quadruped_ctrl/QuadrupedCmd.h"
#include <yaml-cpp/yaml.h>

using namespace BT;


namespace BT {
    typedef std::vector<std::string> LTLState;
    typedef std::vector<std::vector<std::string>> LTLState_Sequence;
    typedef std::vector<std::string> LTLAction_Sequence;
    typedef std::vector<std::string> Locomotion_status;

    template <> inline LTLState convertFromString(StringView str)
    {

        // Vector of strings separated by semicolons
        auto parts = splitString(str, ';');
        LTLState output;
        output.reserve( parts.size() );
        for(auto & part : parts){
            output.push_back(std::string(part.data(), part.size()));
        }

        return output;
    }

    template <> inline LTLState_Sequence convertFromString(StringView str)
    {

        // Customized splitting from: "a,b,c;d,e,f"
        auto parts = splitString(str, ';');
        LTLState_Sequence output;
        LTLState output_state;
        for(auto & part : parts){
            auto part_0 = splitString(part, ',');
            for(auto & part_00 : part_0){
                output_state.push_back(std::string(part_00.data(), part_00.size()));
            }
            output.push_back(output_state);
        }

        return output;
    }

} //end namespace BT

namespace BTNav {
class LTLPreCheck: public ConditionNode
{
public:
    LTLPreCheck(const std::string& name, const NodeConfiguration& config) : ConditionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<BT::LTLState>("ltl_state_current"), InputPort<BT::LTLState_Sequence>("ltl_state_desired_sequence"),
                 BidirectionalPort<BT::LTLState_Sequence>("ltl_state_executed_sequence")};
    }

    NodeStatus tick() override
    {

//        auto int_1 = getInput<int>("in_arg1");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        auto desired_state_seq = getInput<BT::LTLState_Sequence>("ltl_state_desired_sequence");
        auto ltl_state_seq_executed = getInput<BT::LTLState_Sequence>("ltl_state_executed_sequence");
        if(!desired_state_seq || !current_state || !ltl_state_seq_executed) {
            std::cout << name() << ": Fetch ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
        auto desired_state = desired_state_seq.value()[0];
        if(desired_state == current_state.value()){
            // If only have synchronization or stay action; directly pass the check
//            if(desired_state_seq.value().size() == 2 && desired_state == desired_state_seq.value()[desired_state_seq.value().size()-1]){
//                return NodeStatus::SUCCESS;
//            }

            std::cout << name() << ": Check ltl state: " << "SUCCESS" << std::endl;
            std::cout << "CURRENT: ";
            for(const auto& curr_state : current_state.value()){
                 std::cout << curr_state << " ";
            }
            std::cout << std::endl << std::endl;

            // Push the current state to the state history
            ltl_state_seq_executed.value().push_back(current_state.value());
            setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());

            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Check ltl state: " << "FAILED" << std::endl;
            std::cout << "GET: ";
            for(const auto& curr_state : current_state.value()){
                std::cout << curr_state << " ";
            }
            std::cout << std::endl;
            std::cout << "DESIRED: ";
            for(const auto& des_state : desired_state){
                std::cout << des_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }

};

class LocomotionStatusCheck: public ConditionNode
{
public:
    LocomotionStatusCheck(const std::string& name, const NodeConfiguration& config) : ConditionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<BT::Locomotion_status>("locomotion_status")};
    }

    NodeStatus tick() override
    {

//        auto int_1 = getInput<int>("in_arg1");
        auto loco_status = getInput<BT::Locomotion_status>("locomotion_status");
        if(!loco_status || loco_status.value()[1] == "NONE") {
            std::cout << name() << ": Fetch locomotion status: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
        if(loco_status.value()[1] == "NORMAL" && loco_status.value()[0] == "LOCOMOTION"){
            std::cout << name() << ": Check locomotion operating mode: " << "SUCCESS" << std::endl;
            std::cout << "CURRENT: ";
            for(const auto& curr_state : loco_status.value()){
                std::cout << curr_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Check locomotion operating mode: " << "FAILED" << std::endl;
            std::cout << "CURRENT: ";
            for(const auto& curr_state : loco_status.value()){
                std::cout << curr_state << " ";
            }
            std::cout << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }

};

class ReactiveLTLStateCheck: public ConditionNode
{
public:
    ReactiveLTLStateCheck(const std::string& name, const NodeConfiguration& config) : ConditionNode(name, config){}

    static PortsList  providedPorts(){
        return { InputPort<BT::LTLState>("ltl_state_current"), InputPort<BT::LTLState_Sequence>("ltl_state_desired_sequence")};
    }

    NodeStatus tick() override
    {

        //        auto int_1 = getInput<int>("in_arg1");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        auto desired_state_seq = getInput<BT::LTLState_Sequence>("ltl_state_desired_sequence");
        if(!desired_state_seq || !current_state) {
            std::cout << name() << ": Fetch ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

        // check the second state which is reactive LTL state such as loaded/standby
        auto desired_reactive_state = desired_state_seq.value()[0][1];
        auto current_reactive_state = current_state.value()[1];
        if(desired_reactive_state == current_reactive_state){
            std::cout << name() << ": Check reactive ltl state: " << "SUCCESS" << std::endl;
            std::cout << "CURRENT: ";
            std::cout << current_reactive_state << " ";
            std::cout << std::endl << std::endl;

            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Check reactive ltl state: " << "FAILED" << std::endl;
            std::cout << "GET: ";
            std::cout << current_reactive_state << " ";
            std::cout << std::endl;
            std::cout << "DESIRED: ";
            std::cout << desired_reactive_state << " ";
            std::cout << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }

};

class UpdateLTL : public SyncActionNode
{
public:
    UpdateLTL(const std::string& name, const NodeConfiguration& config, const YAML::Node& transition_system) :
              SyncActionNode(name, config),
              transition_system_(transition_system) {}

    static PortsList  providedPorts(){
        return {BidirectionalPort<BT::LTLAction_Sequence>("action_sequence"),
                BidirectionalPort<BT::LTLState_Sequence>("ltl_state_desired_sequence"),
                BidirectionalPort<BT::LTLAction_Sequence>("action_sequence_executed"),
                OutputPort<std::string>("current_action"),
                OutputPort<std::string>("bt_action_type")};
    }

    NodeStatus tick() override {
        auto action_sequence = getInput<BT::LTLAction_Sequence>("action_sequence");
        auto ltl_state_seq = getInput<BT::LTLState_Sequence>("ltl_state_desired_sequence");
        auto action_sequence_executed = getInput<BT::LTLAction_Sequence>("action_sequence_executed");
        if(action_sequence && ltl_state_seq && action_sequence_executed) {
            BT::LTLAction_Sequence act_seq = action_sequence.value();
            BT::LTLState_Sequence state_seq = ltl_state_seq.value();
            BT::LTLAction_Sequence act_seq_executed = action_sequence_executed.value();
            act_seq_executed.push_back(act_seq.front());
            act_seq.erase(act_seq.begin());
            state_seq.erase(state_seq.begin());

            setOutput<BT::LTLState_Sequence>("ltl_state_desired_sequence", state_seq);
            setOutput<BT::LTLAction_Sequence>("action_sequence", act_seq);
            setOutput<BT::LTLAction_Sequence>("action_sequence_executed", act_seq_executed);
            setOutput<std::string>("current_action", act_seq[0]);

            bool sanity_check1 = false;
            YAML::Node action_dict;
            std::string bt_action_type;
            // Check the first action to be executed
            for (YAML::const_iterator iter = transition_system_["actions"].begin();
                 iter != transition_system_["actions"].end(); ++iter) {
                if (iter->first.as<std::string>() == act_seq[0]) {
                    action_dict = transition_system_["actions"][iter->first.as<std::string>()];
                    bt_action_type = action_dict["type"].as<std::string>();
                    sanity_check1 = true;
                    setOutput<std::string>("bt_action_type", bt_action_type);
                    break;
                }
            }

            if (!sanity_check1) {
                std::cout << "The first action from LTL planner not found in transition system" << std::endl << std::endl;
                return NodeStatus::FAILURE;
            }

            std::cout << name() << ": Update ltl state: " << "SUCCESS" << std::endl << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": Update ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
    }

private:
    YAML::Node transition_system_;
};

class ReplanningRequestLevel1 : public SyncActionNode
{
public:
    ReplanningRequestLevel1(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){}

    static PortsList  providedPorts(){
        return {BidirectionalPort<int>("replanning_request"),
                BidirectionalPort<BT::LTLState_Sequence>("ltl_state_executed_sequence"),
                InputPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override {
        auto replanning_request = getInput<int>("replanning_request");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        auto ltl_state_seq_executed = getInput<BT::LTLState_Sequence>("ltl_state_executed_sequence");
        if(!current_state || !ltl_state_seq_executed) {
            std::cout << name() << ": Fetch ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

        if(replanning_request && replanning_request.value() == 0) {
            setOutput<int>("replanning_request", 1);
            std::cout << name() << ": replanning request Level 1" << " submitted: SUCCESS" << std::endl << std::endl;
            if(ltl_state_seq_executed.value().empty()){
                // Push the current state to the state history
                ltl_state_seq_executed.value().push_back(current_state.value());
                setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());
            } else if(current_state.value() != ltl_state_seq_executed.value().back()){
                // Push the current state to the state history
                ltl_state_seq_executed.value().push_back(current_state.value());
                setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());
            }
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": replanning request " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
    }
};

class ReplanningRequestLevel2 : public SyncActionNode
{
public:
    ReplanningRequestLevel2(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){}

    static PortsList  providedPorts(){
        return {BidirectionalPort<int>("replanning_request"),
                BidirectionalPort<BT::LTLState_Sequence>("ltl_state_executed_sequence"),
                InputPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override {
        auto replanning_request = getInput<int>("replanning_request");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        auto ltl_state_seq_executed = getInput<BT::LTLState_Sequence>("ltl_state_executed_sequence");
        if(!current_state || !ltl_state_seq_executed) {
            std::cout << name() << ": Fetch ltl state: " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

        if(replanning_request && replanning_request.value() == 0) {
            setOutput<int>("replanning_request", 2);
            std::cout << name() << ": replanning request Level 2" << " submitted: SUCCESS" << std::endl << std::endl;
            if(ltl_state_seq_executed.value().empty()){
                // Push the current state to the state history
                ltl_state_seq_executed.value().push_back(current_state.value());
                setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());
            } else if(current_state.value() != ltl_state_seq_executed.value().back()){
                // Push the current state to the state history
                ltl_state_seq_executed.value().push_back(current_state.value());
                setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());
            }
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": replanning request " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
    }
};

class ReplanningRequestLevel3 : public SyncActionNode
{
public:
    ReplanningRequestLevel3(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){}

    static PortsList  providedPorts(){
        return {BidirectionalPort<int>("replanning_request"),
                BidirectionalPort<BT::LTLState_Sequence>("ltl_state_executed_sequence"),
                InputPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override {
        auto replanning_request = getInput<int>("replanning_request");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        auto ltl_state_seq_executed = getInput<BT::LTLState_Sequence>("ltl_state_executed_sequence");
        if(replanning_request && replanning_request.value() == 0) {
            setOutput<int>("replanning_request", 3);
            std::cout << name() << ": replanning request Level 3" << " submitted: SUCCESS" << std::endl << std::endl;
            if(ltl_state_seq_executed.value().empty()){
                // Push the current state to the state history
                ltl_state_seq_executed.value().push_back(current_state.value());
                setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());
            } else if(current_state.value() != ltl_state_seq_executed.value().back()){
                // Push the current state to the state history
                ltl_state_seq_executed.value().push_back(current_state.value());
                setOutput<BT::LTLState_Sequence>("ltl_state_executed_sequence", ltl_state_seq_executed.value());
            }
            return NodeStatus::SUCCESS;
        } else {
            std::cout << name() << ": replanning request " << "FAILED" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }
    }
};

class MoveAction : public CoroActionNode
{
public:
    MoveAction(const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<bool>("move_base_finished"), InputPort<bool>("move_base_idle"),
                 InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 OutputPort<bool>("goal_sent")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        if(!current_action || current_action.value() == "NONE" || !bt_action_type || bt_action_type.value() == "NONE"){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "move") {
            std::cout << name() << ": MOVE_COMMAND: " << current_action.value() << " Yield" << std::endl << std::endl;
            setOutput<bool>("goal_sent", false);
            setStatusRunningAndYield();
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

        while (true)
        {
            auto move_base_idle = getInput<bool>("move_base_idle");

            auto move_base_finished = getInput<bool>("move_base_finished");
            if (move_base_finished && move_base_finished.value())
            {
                std::cout << name() << ": move_base is finidshed: SUCCESS" << std::endl << std::endl;
                setOutput<bool>("goal_sent", true);
                return BT::NodeStatus::SUCCESS;
            }

            if (move_base_idle && move_base_idle.value())
            {
                std::cout << name() << ": move_base is idle: FAILURE" << std::endl << std::endl;
                setOutput<bool>("goal_sent", true);
                return BT::NodeStatus::FAILURE;
            }

            setOutput<bool>("goal_sent", true);
            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << this->name() << getInput<std::string>("current_action").value() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

};

class StayAction : public SyncActionNode
{
public:
    StayAction(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 BidirectionalPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        if(!current_action || current_action.value() == "NONE" ||
           !bt_action_type || bt_action_type.value() == "NONE" ||
           !current_state){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "stay") {
            // Do nothing
            std::cout << name() << ": STAY" << current_action.value() << " Yield" << std::endl << std::endl;
            setOutput<BT::LTLState>("ltl_state_current", current_state.value());
            return NodeStatus::SUCCESS;
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }
};

class SynchronizedTransitionAction : public SyncActionNode
{
public:
    SynchronizedTransitionAction(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 BidirectionalPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        if(!current_action || current_action.value() == "NONE" ||
           !bt_action_type || bt_action_type.value() == "NONE" ||
           !current_state){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "synchronized_transition") {
            // Do nothing
            std::cout << name() << ": Synchronized transition" << current_action.value() << " Yield" << std::endl << std::endl;
            setOutput<BT::LTLState>("ltl_state_current", current_state.value());
            return NodeStatus::SUCCESS;
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }
};

class PickAction : public SyncActionNode
{
public:
    PickAction(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 BidirectionalPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        if(!current_action || current_action.value() == "NONE" ||
        !bt_action_type || bt_action_type.value() == "NONE" ||
        !current_state){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "pick") {
            // Do nothing
            std::cout << name() << ": Pick action" << current_action.value() << " Yield" << std::endl << std::endl;
            auto updated_state = current_state.value();
            if(updated_state[1] != "standby"){
                std::cout << "Current load state ERROR; shouldn't happen" << std::endl;
                return NodeStatus::FAILURE;
            }
            updated_state[1] = "loaded";
            setOutput<BT::LTLState>("ltl_state_current", updated_state);
            return NodeStatus::SUCCESS;
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }
};

class DropAction : public SyncActionNode
{
public:
    DropAction(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 BidirectionalPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        if(!current_action || current_action.value() == "NONE" ||
        !bt_action_type || bt_action_type.value() == "NONE" ||
        !current_state){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "drop") {
            // Do nothing
            std::cout << name() << ": Drop action" << current_action.value() << " Yield" << std::endl << std::endl;
            auto updated_state = current_state.value();
            if(updated_state[1] != "loaded"){
                std::cout << "Current load state ERROR; shouldn't happen" << std::endl;
                return NodeStatus::FAILURE;
            }
            updated_state[1] = "standby";
            setOutput<BT::LTLState>("ltl_state_current", updated_state);
            return NodeStatus::SUCCESS;
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }
};

class GuideAction : public SyncActionNode
{
public:
    GuideAction(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 BidirectionalPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        if(!current_action || current_action.value() == "NONE" ||
        !bt_action_type || bt_action_type.value() == "NONE" ||
        !current_state){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "start_training") {
            // Do nothing
            std::cout << name() << ": Start guide mode" << current_action.value() << " Yield" << std::endl << std::endl;
            auto updated_state = current_state.value();
            if(updated_state[1] != "standby"){
                std::cout << "Current environment-independent robot state ERROR; shouldn't happen" << std::endl;
                return NodeStatus::FAILURE;
            }
            updated_state[1] = "training";
            setOutput<BT::LTLState>("ltl_state_current", updated_state);
            return NodeStatus::SUCCESS;
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }
};

class BackNormalAction : public SyncActionNode
{
public:
    BackNormalAction(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("current_action"), InputPort<std::string>("bt_action_type"),
                 BidirectionalPort<BT::LTLState>("ltl_state_current")};
    }

    NodeStatus tick() override
    {
        auto current_action = getInput<std::string>("current_action");
        auto bt_action_type = getInput<std::string>("bt_action_type");
        auto current_state = getInput<BT::LTLState>("ltl_state_current");
        if(!current_action || current_action.value() == "NONE" ||
        !bt_action_type || bt_action_type.value() == "NONE" ||
        !current_state){
            return NodeStatus::FAILURE;
        }

        if(bt_action_type.value() == "terminate_training") {
            // Do nothing
            std::cout << name() << ": Start normal mode" << current_action.value() << " Yield" << std::endl << std::endl;
            auto updated_state = current_state.value();
            if(updated_state[1] != "training"){
                std::cout << "Current environment-independent robot state ERROR; shouldn't happen" << std::endl;
                return NodeStatus::FAILURE;
            }
            updated_state[1] = "standby";
            setOutput<BT::LTLState>("ltl_state_current", updated_state);
            return NodeStatus::SUCCESS;
        }else{
            std::cout << name() << ": Wrong action type; Check the switch node" << std::endl << std::endl;
            return NodeStatus::FAILURE;
        }

    }
};

class RecoveryStand : public CoroActionNode
{
public:
    RecoveryStand (const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<BT::Locomotion_status>("locomotion_status")};
    }

    NodeStatus tick() override
    {
        auto loco_status = getInput<BT::Locomotion_status>("locomotion_status");
        std::string current_fsm = loco_status.value()[0];
        std::string ope_mode = loco_status.value()[1];
        if(!loco_status || current_fsm == "NONE" || ope_mode == "NONE"){
            return NodeStatus::FAILURE;
        }

//        setOutput<std::string>("action", "MOVE_COMMAND");
        quadruped_ctrl::QuadrupedCmd cmd;
        cmd.request.cmd = 0;
        if(ros::service::call("ControlMode", cmd)){
            std::cout << name() << ": Sent control mode signal 0: " << std::endl << std::endl;
        }
        std::cout << name() << ": Try recovery stand: " << " Yield" << std::endl << std::endl;
        setStatusRunningAndYield();

        while (true)
        {
            loco_status = getInput<BT::Locomotion_status>("locomotion_status");
            current_fsm = loco_status.value()[0];
            ope_mode = loco_status.value()[1];

            if (loco_status && current_fsm == "STAND_UP" && ope_mode == "NORMAL")
            {
                std::cout << name() << ": recovery standup is finidshed: SUCCESS" << std::endl << std::endl;
                return BT::NodeStatus::SUCCESS;
            }

            if (!loco_status)
            {
                std::cout << name() << ": recovery standup is: FAILURE" << std::endl << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << this->name() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

//private:
//    ros::NodeHandle n_;
//    ros::ServiceClient client;

};

class LocomotionStart : public CoroActionNode
{
public:
    LocomotionStart (const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<BT::Locomotion_status>("locomotion_status")};
    }

    NodeStatus tick() override
    {
        auto loco_status = getInput<BT::Locomotion_status>("locomotion_status");
        std::string current_fsm = loco_status.value()[0];
        std::string ope_mode = loco_status.value()[1];
        if(!loco_status || current_fsm == "NONE" || ope_mode == "NONE"){
            return NodeStatus::FAILURE;
        }

        if(ope_mode == "ESTOP"){
            std::cout << "SHOULDN'T HAPPEN! SOMETHING WRONG WITH PREVIOUS NODES" << std::endl;
            return NodeStatus::FAILURE;
        }

        quadruped_ctrl::QuadrupedCmd cmd;
        cmd.request.cmd = 2;
        if(ros::service::call("ControlMode", cmd)){
            std::cout << name() << ": Sent control mode signal 2: " << std::endl << std::endl;
        }
        std::cout << name() << ": Try start locomotion: " << " Yield" << std::endl << std::endl;
        setStatusRunningAndYield();

        while (true)
        {
            loco_status = getInput<BT::Locomotion_status>("locomotion_status");
            current_fsm = loco_status.value()[0];
            ope_mode = loco_status.value()[1];

            if (loco_status && current_fsm == "LOCOMOTION" && ope_mode == "NORMAL")
            {
                std::cout << name() << ": locomotion bootup is finidshed: SUCCESS" << std::endl << std::endl;
                return BT::NodeStatus::SUCCESS;
            }

            if (!loco_status || ope_mode == "ESTOP")
            {
                std::cout << name() << ": locomotion bootup is: FAILURE" << std::endl << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            setStatusRunningAndYield();
        }
    }

    void halt() override
    {
        std::cout << this->name() << ": halt" << std::endl;
        CoroActionNode::halt();
    }

};

class FakeDetectionLevel1 : public SyncActionNode {
public:
    FakeDetectionLevel1(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("replanning_fake_input") };
    }

    NodeStatus tick() override
    {
        auto fake_input = getInput<int>("replanning_fake_input");
        if(!fake_input){
            return NodeStatus::FAILURE;
        }

        if(fake_input.value() == 1) {
            std::cout << "Received fake replanning request level 1 from user" << std::endl;
            return NodeStatus::FAILURE;
        }else{
            return NodeStatus::SUCCESS;
        }

    }
};

class FakeDetectionLevel2 : public SyncActionNode {
public:
    FakeDetectionLevel2(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("replanning_fake_input") };
    }

    NodeStatus tick() override
    {
        auto fake_input = getInput<int>("replanning_fake_input");
        if(!fake_input){
            return NodeStatus::FAILURE;
        }

        if(fake_input.value() == 2) {
            std::cout << "Received fake replanning request level 2 from user" << std::endl;
            return NodeStatus::FAILURE;
        }else{
            return NodeStatus::SUCCESS;
        }

    }
};

class FakeDetectionLevel3 : public SyncActionNode {
public:
    FakeDetectionLevel3(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("replanning_fake_input") };
    }

    NodeStatus tick() override
    {
        auto fake_input = getInput<int>("replanning_fake_input");
        if(!fake_input){
            return NodeStatus::FAILURE;
        }

        if(fake_input.value() == 3) {
            std::cout << "Received fake replanning request level 3 from user" << std::endl;
            return NodeStatus::FAILURE;
        }else{
            return NodeStatus::SUCCESS;
        }

    }
};
} // end namespace BTNav

#endif //LTL_AUTOMATION_A1_NAVIGATION_NODE_H
