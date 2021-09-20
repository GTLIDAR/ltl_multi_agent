#!/usr/bin/env python
import rospy
from ltl_automaton_msgs.srv import TrapCheck, TrapCheckResponse
from networkx import dijkstra_predecessor_and_distance, has_path
from ltl_automaton_planner.ltl_automaton_utilities import handle_ts_state_msg

#===================================================
#           Trap State Detection Plugin
#===================================================
# Provide a check for trap service. The service will
# test a TS state and returns if reaching it will 
#   create a violation of the hard task or not.
#===================================================
class TrapDetectionPlugin(object):
    #--------------------------------------------------------------------------
    # Plugin object constructor, must have as argument: ltl_planner, args_dict
    #--------------------------------------------------------------------------
    def __init__(self, ltl_planner, args_dict):
        self.ltl_planner = ltl_planner

    #-------------------------------------------
    # Initialized afer constructor by main code
    #-------------------------------------------
    def init(self):
        None

    #--------------------------------------
    # Setup ROS subscribers and publishers
    #--------------------------------------
    def set_sub_and_pub(self):
        # Initialize check for trap service
        self.trap_srv = rospy.Service('check_for_trap', TrapCheck, self.trap_check_callback)

    #------------------------------
    # Run at every TS state update
    #------------------------------
    def run_at_ts_update(self, ts_state):
        None

    #---------------------------------------------
    # Callback for checking is given TS is a trap
    #---------------------------------------------
    def trap_check_callback(self, trap_check_req):
        # Extract state from request
        ts_state = handle_ts_state_msg(trap_check_req.ts_state)

        # Create response message
        res = TrapCheckResponse()

        # Check if TS state is trap
        is_trap = self.is_trap(ts_state)

        # TS state is trap
        if is_trap == 1:
            res.is_trap = True
            res.is_connected = True
        # TS state is not a trap
        elif is_trap == -1:
            res.is_trap = False
            res.is_connected = True
        # Error: TS state is not connected to current state
        else:
            res.is_trap = False
            res.is_connected = False

        # Return service response
        return res

    #---------------------------------
    # Check if given TS state in trap 
    #---------------------------------
    # if reached, no possible path to accept
    def is_trap(self, ts_state):
        # Get possible states if current states were to be updated from a given TS state
        reachable_states = self.ltl_planner.product.get_possible_states(ts_state)

        # If reachable states exist
        if reachable_states:
            # If TS state is trap
            if self.check_possible_states_for_trap(reachable_states):
                return 1
            # If TS state is not a trap
            else:
                return -1
        # If no reachables states, TS is not connected to current state
        else:
            return 0

    #--------------------------------------------------------------
    # Check if a possible state set has a path to accepting states
    #--------------------------------------------------------------
    # Returns True if possible state set is from a trap state
    # (meaning there are no path to accepting from trap possible state set)
    def check_possible_states_for_trap(self, possible_state_set):
        for s in possible_state_set:
            if self.has_path_to_accept_with_cycle(s):
                return False              
        return True

    def has_path_to_accept(self, f_s):
        for acc in self.ltl_planner.product.graph['accept']:
            if has_path(self.ltl_planner.product, f_s, acc):
                return True
        return False

    def has_path_to_accept_with_cycle(self, f_s):
        for acc in self.ltl_planner.product.graph['accept_with_cycle']:
            if has_path(self.ltl_planner.product, f_s, acc):
                return True
        return False