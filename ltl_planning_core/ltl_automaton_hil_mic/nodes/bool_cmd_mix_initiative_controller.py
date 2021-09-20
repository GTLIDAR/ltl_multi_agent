#!/usr/bin/env python
import sys
import rospy
from copy import deepcopy
from std_msgs.msg import Bool

# Import transition system loader
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file

#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState
from ltl_automaton_msgs.srv import TrapCheck, TrapCheckRequest

#=============================================
#       Mix Initative Controller object
#            for action commands
#                     ---
#  Take as input an action name (String msg)
#  to test the potential resulting state as
#   trap or not and output either planner or
#           human action command
#=============================================
class BoolCmdMixer(object):
    def __init__(self):
        # Get parameters
        self.load_params()

        # Setup subscribers and publishers
        self.set_pub_sub()

        rospy.loginfo("Human-in-the-loop boolean commands mix-initiative controller initialized")

    def load_params(self):
        """
        Load controller parameters from parameter server.
        """

        self.curr_ts_state = None

        # Get TS from param
        self.transition_system = import_ts_from_file(rospy.get_param('transition_system_textfile'))

        # Get monitored TS state model
        self.state_dimension_name = rospy.get_param("~state_dimension_name", "load")

        # Get monitored action
        self.monitored_action = rospy.get_param("~monitored_action", "pick")
        
        # Create dict to retrieve next state given current state and next action
        self.action_to_state = dict()
        for state in self.transition_system['state_models'][self.state_dimension_name]['nodes']:
            temp_dict = dict()
            for connected_state in self.transition_system['state_models'][self.state_dimension_name]['nodes'][state]['connected_to']:
                temp_dict.update({self.transition_system['state_models'][self.state_dimension_name]['nodes'][state]['connected_to'][connected_state]: connected_state})
            self.action_to_state.update({state: temp_dict})

    def set_pub_sub(self):
        """
        Setup subscribers, publishers and service clients.
        """

        # Set trap check service client
        self.trap_cheq_srv = rospy.ServiceProxy("check_for_trap", TrapCheck)

        # Set mix initiave controller output
        self.mix_cmd_pub = rospy.Publisher("mix_cmd", Bool, queue_size=50)

        # Set agent TS state subscriber
        rospy.Subscriber("ts_state", TransitionSystemStateStamped, self.ts_state_callback, queue_size=50)

        # Set human input planner
        rospy.Subscriber("key_cmd", Bool, self.teleop_cmd_callback, queue_size=50)

        # Set planner input subscriber
        rospy.Subscriber("planner_cmd", Bool, self.planner_cmd_callback, queue_size=50)


    def ts_state_callback(self, msg):
        """
        Agent TS state callback.
        """

        # If not using same state model type, print warning and ignore message
        if not self.state_dimension_name in msg.ts_state.state_dimension_names:
            rospy.logdebug("Received TS state does not include state model type used by boolean command HIL MIC (%s), TS state is of type %s"
                          % (self.state_dimension_name, msg.ts_state.state_dimension_names))
        # If lenght of states is different from length of state dimension names, message is malformed
        # print warning and ignore message
        if not (len(msg.ts_state.state_dimension_names) == len(msg.ts_state.states)):
            rospy.logdebug("Received TS state but number of states: %i doesn't correpond to number of state dimensions: %i"
                          % (len(msg.ts_state.states),len(msg.ts_state.state_dimension_names)))
        # Else message is valid, save it
        else:
            self.curr_ts_state = msg.ts_state


    def planner_cmd_callback(self, msg):
        """
        Planner command input callback.
        """

        # Use planner input directly
        self.mix_cmd_pub.publish(msg)


    def teleop_cmd_callback(self, msg):
        """
        Human command input callback
        """

        # If boolean command is true
        if msg.data:
            # Get next state if action is executed
            for i in range(len(self.curr_ts_state.state_dimension_names)):
                # Find state in the TS state
                if self.curr_ts_state.state_dimension_names[i] == self.state_dimension_name:
                    # Check if trap using potential state if action would to be executed
                    if not self.check_for_trap(self.action_to_state[self.curr_ts_state.states[i]][self.monitored_action]):
                        # If not a trap publish command
                        self.mix_cmd_pub.publish(msg)

            # If TS state doesn't contain proper dimension
            return None

    def check_for_trap(self, potential_state):
        """
        Check if risk of trap when executing commands.
        """

        # Create check for trap request from TS state
        ts_state_to_check = deepcopy(self.curr_ts_state)
        for i in range(len(self.curr_ts_state.state_dimension_names)):
            # Replace current region by region to check
            if self.curr_ts_state.state_dimension_names[i] == self.state_dimension_name:
                ts_state_to_check.states[i] = potential_state

                # Populate request and call service to check if closest region would trigger a trap state
                check_for_trap_req = TrapCheckRequest()
                check_for_trap_req.ts_state = ts_state_to_check
                check_for_trap_res = self.trap_cheq_srv(check_for_trap_req)

                rospy.logwarn("LTL boolean command MIC: testing next state %s" % (ts_state_to_check.states))

                # if either unconnected or a trap (both considered a trap)
                if not check_for_trap_res.is_connected or (check_for_trap_res.is_connected and check_for_trap_res.is_trap):
                    rospy.logwarn("LTL boolean command MIC: Agent is in state %s and state %s is a trap" % (self.curr_ts_state.states, check_for_trap_req.ts_state.states))
                # Return true if a trap
                return True

        # If not a trap or cannot be tested
        return False


#============================
#            Main            
#============================
if __name__ == '__main__':
    rospy.init_node('bool_cmd_hil_mic',anonymous=False)
    try:
        bool_cmd_hil_mic = BoolCmdMixer()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("Boolean Command HIL MIC: %s" %(e))
        sys.exit(0)