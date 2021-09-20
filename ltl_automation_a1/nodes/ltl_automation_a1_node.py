#!/usr/bin/env python
import rospy
import sys
import yaml
import std_msgs
from copy import deepcopy
#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState
# Import transition system loader
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
# Import modules for commanding the a1
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

#=================================================================
#  Interfaces between LTL planner node and lower level controls
#                       -----------------
# The node is reponsible for outputting current agent state based
# on TS and agent output.
# The node converts TS action to interpretable commands using
# action attributes defined in the TS config file
#=================================================================
class LTLControllerA1(object):
    def __init__(self):

        self.init_params()

        self.create_monitors()

        self.set_pub_sub()

        self.main_loop()

    #---------------------------------------------------
    # Get params from ROS param server and config files
    #---------------------------------------------------
    def init_params(self):
        self.agent_name = rospy.get_param('agent_name', "a1_gazebo")

        # Get TS from param
        self.transition_system = import_ts_from_file(rospy.get_param('transition_system_textfile'))

        # Init state message with TS
        self.ltl_state_msg = TransitionSystemStateStamped()
        self.ltl_state_msg.ts_state.state_dimension_names = self.transition_system["state_dim"]

        # Get obstacle name list, if any
        obstacle_name_list = rospy.get_param('~obstacle_names', [])
        self.obstacles = {}
        self.occupied_regions = []
        # if obstacle name exist, initialize dict
        if obstacle_name_list:
            for obstacle_name in obstacle_name_list:
                rospy.loginfo("LTL automaton A1 node: keeping track of potential obstacle "+str(obstacle_name))
                self.obstacles.update({str(obstacle_name): ""})

        # Init feedback variables
        self.pick_box_feedback = False
        self.pick_assembly_feedback = False
        self.deliver_assembly_feedback = False

        # Initialize running time and index of command received and executed
        self.t0 = rospy.Time.now()
        self.t = self.t0
        self.plan_index = 0
        self.next_action = {}

    #-------------------------------------------------------------------------
    # Create monitoring object for every state dimension in transition system
    #-------------------------------------------------------------------------
    def create_monitors(self):
        number_of_dimensions = len(self.transition_system["state_dim"])

        # Init LTL state variables
        self.curr_ltl_state = [None for element in range(number_of_dimensions)]
        self.prev_ltl_state = deepcopy(self.curr_ltl_state)

        # Setup subscribers
        for i in range(number_of_dimensions):
            print("checking dimension states")
            dimension = self.transition_system["state_dim"][i]
            print(dimension)
            if (dimension == "2d_pose_region"):
                # Setup subscriber to 2D pose region monitor
                self.a1_region_sub = rospy.Subscriber("current_region", String, self.region_state_callback, i, queue_size=100)
            elif (dimension == "a1_load"):
                # Setup subscriber to a1 load state
                self.a1_load_state_id = i
                self.curr_ltl_state[i] = "unloaded"
                #self.a1_load_sub = rospy.Subscriber("current_load_state", String, self.load_state_callback, i, queue_size=100)
            else:
                raise ValueError("state type [%s] is not supported by LTL a1" % (dimension))

    #----------------------------------
    # Setup subscribers and publishers
    #----------------------------------
    def set_pub_sub(self):
        # Setup navigation commands for a1
        self.navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("LTL automaton a1 node: waiting for a1 move base action server...")
        self.navigation.wait_for_server() # wait for action server to start

        # Station access request publisher
        self.station_access_request_pub = rospy.Publisher("station_access_request", std_msgs.msg.String, latch=True, queue_size=1)

        # Setup LTL state publisher
        self.ltl_state_pub = rospy.Publisher("ts_state", TransitionSystemStateStamped, latch=True, queue_size=10)

        # Setup subscriber to ltl_automaton_core next_move_cmd
        self.next_move_sub = rospy.Subscriber("next_move_cmd", std_msgs.msg.String, self.next_move_callback, queue_size=1)

        # Setup assembly task (pick box) acknowkedge topic subscriber
        self.pick_box_feedback_sub = rospy.Subscriber("/placed_box_ack", std_msgs.msg.Bool, self.pick_box_feedback_callback)

        # Setup assembly task (pick assembly) acknowkedge topic subscriber
        self.pick_assembly_feedback_sub = rospy.Subscriber("/placed_assembly_ack", std_msgs.msg.Bool, self.pick_assembly_feedback_callback)

        # Setup delivery task acknowledge topic subscriber
        self.deliver_assembly_feedback_sub = rospy.Subscriber("/delivered_assembly_ack", std_msgs.msg.Bool, self.deliver_assembly_feedback_callback)

        # Setup obstacle region subcribers
        for obstacle_name in self.obstacles.keys():
            rospy.Subscriber("/"+obstacle_name+"/current_region", String, self.obstacle_region_callback, obstacle_name, queue_size=1)

        rospy.loginfo("LTL automaton A1 node: initialized!")

    #----------------------------------------
    # Handle message from load state monitor
    #----------------------------------------
    def load_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data

    #------------------------------------------
    # Handle message from region state monitor
    #------------------------------------------
    def region_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data
        print("received region "+str(self.curr_ltl_state[id]))

    #------------------------------------------------
    # Handle message from obstacle region monitoring
    #------------------------------------------------
    def obstacle_region_callback(self, msg, obstacle_name):
        # Get occupied region for the obstacle
        self.obstacles[obstacle_name] = [msg.data]

        # Check if region is a station, and add connected cells if it is the case
        if self.transition_system['state_models']['2d_pose_region']['nodes'][msg.data]['attr']['type'] == 'station':
            for connected_cell in self.transition_system['state_models']['2d_pose_region']['nodes'][msg.data]['connected_to'].keys():
                self.obstacles[obstacle_name].append(str(connected_cell))

        # Rebuild occupied regions list
        self.occupied_regions = []
        for reg_list in self.obstacles.values():
            # If list not empty
            if reg_list:
                self.occupied_regions = self.occupied_regions+reg_list

    #---------------------------------
    # Handle pick box acknowledgement
    #---------------------------------
    def pick_box_feedback_callback(self, msg):
        self.pick_box_feedback = msg.data

    #--------------------------------------
    # Handle pick assembly acknowledgement
    #--------------------------------------
    def pick_assembly_feedback_callback(self, msg):
        self.pick_assembly_feedback = msg.data

    #--------------------------------------
    # Handle pick assembly acknowledgement
    #--------------------------------------
    def deliver_assembly_feedback_callback(self, msg):
        self.deliver_assembly_feedback = msg.data

    #---------------------------------------
    # Handle next move command from planner
    #---------------------------------------
    def next_move_callback(self, msg):
        '''Recieve next_move_cmd from ltl_automaton_core planner and convert into robot action to implement'''

        # Update running time and augment plan index
        self.t = rospy.Time.now()-self.t0
        self.plan_index += 1

        # Extract command message string
        cmd_str =  msg.data
        action_dict = None

        # Check if next_move_cmd is 'None', which is output by ltl_automaton_core if the current state is not in the TS
        if cmd_str == "None":

            # To do: Handle when ltl_automaton_core encounteres state outside of TS (i.e. next_move_cmd = 'None')
            rospy.logwarn('None next_move_cmd sent to LTL A1')
        else:

            # Check if next_move_cmd is in list of actions from transition_system
            for act in self.transition_system['actions']:
                if str(act) == cmd_str:

                    # Extract action types, attributes, etc. in dictionary
                    action_dict = self.transition_system['actions'][str(act)]

                    break

            # Raise error if next_move_cmd does not exist in transition system
            if not(action_dict):
                raise ValueError("next_move_cmd not found in LTL a1 transition system")


        # Reset feedbacks
        self.pick_box_feedback = False
        self.pick_assembly_feedback = False
        self.deliver_assembly_feedback = False

        # Send action_dict to a1_action()
        self.next_action = action_dict

    #----------------------
    # Execute given action
    #----------------------
    def a1_action(self, act_dict):
        '''Read components of act_dict associated with current command and output control to a1'''

        #--------------
        # Move command
        #--------------
        if act_dict['type'] == 'move':
            # Extract pose to move to:
            pose = act_dict['attr']['pose']
            region = act_dict['attr']['region']

            # Check if region is already occupied
            if region in self.occupied_regions:
                # Region is already occupied, wait before moving
                return False

            # If next region is a station, request access, otherwise empty station request
            station_access_req = std_msgs.msg.String()
            if self.transition_system['state_models']['2d_pose_region']['nodes'][region]['attr']['type'] == "station":
                station_access_req.data = region
            self.station_access_request_pub.publish(station_access_req)

            # Set new navigation goal and send
            GoalMsg = MoveBaseGoal()
            GoalMsg.target_pose.header.seq = self.plan_index
            GoalMsg.target_pose.header.stamp = self.t
            GoalMsg.target_pose.header.frame_id = 'map'
            GoalMsg.target_pose.pose.position.x = pose[0][0]
            GoalMsg.target_pose.pose.position.y = pose[0][1]
            #quaternion = quaternion_from_euler(0, 0, goal[2])
            GoalMsg.target_pose.pose.orientation.x = pose[1][0]
            GoalMsg.target_pose.pose.orientation.y = pose[1][1]
            GoalMsg.target_pose.pose.orientation.z = pose[1][2]
            GoalMsg.target_pose.pose.orientation.w = pose[1][3]
            self.navigation.send_goal(GoalMsg)

            return True

        #---------------------------
        # Pick box at assembly line
        #---------------------------
        if act_dict['type'] == 'pick_box':
            # Wait for acknowledgement of placed box on a1
            # return false until feedback as been received
            if not self.pick_box_feedback:
                return False
            else:
                # Reset feedback flag
                self.pick_box_feedback = False
                # Change state
                self.curr_ltl_state[self.a1_load_state_id] = "loaded_box"
                print("PICK ACTION OK, CHANGE TO LOADED BOX")
                return True
            
        #-------------------------------------
        # Pick full assembly at assembly line
        #-------------------------------------
        if act_dict['type'] == 'pick_assembly':
            # Wait for acknowledgement of placed assembly on a1
            # return false until feedback as been received
            if not self.pick_assembly_feedback:
                return False
            else:
                # Reset feedback flag
                self.pick_assembly_feedback = False
                # Change state
                self.curr_ltl_state[self.a1_load_state_id] = "loaded_assembly"
                print("PICK ASSEMBLY ACTION OK, CHANGE TO LOADED ASSEMBLY")
                return True

        #-----------------------
        # Deliver full assembly
        #-----------------------
        if act_dict['type'] == 'deliver_assembly':
            # Wait for acknowledgement of assembly delivery from a1
            # return false until feedback as been received
            if not self.deliver_assembly_feedback:
                return False
            else:
                # Reset feedback flag
                self.deliver_assembly_feedback = False
                # Change state
                self.curr_ltl_state[self.a1_load_state_id] = "unloaded"
                print("DELIVER ACTION OK, CHANGE TO UNLOADED")
                return True

    #-----------
    # Main loop
    #-----------
    def main_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # If current state is different from previous state
            # update message and publish it
            if not (self.curr_ltl_state == self.prev_ltl_state):
                # Update previous state
                self.prev_ltl_state = deepcopy(self.curr_ltl_state)
                # If all states are initialized (not None), publish message
                if all([False for element in self.curr_ltl_state if element == None]):
                    # Publish msg
                    self.ltl_state_msg.header.stamp = rospy.Time.now()
                    self.ltl_state_msg.ts_state.states = self.curr_ltl_state
                    self.ltl_state_pub.publish(self.ltl_state_msg)

            # If waiting for obstacles or acknowledgement, check again
            if self.next_action:
                # If action returns true, action was carried out and is reset
                if self.a1_action(self.next_action):
                    self.a1_action = {}
                    
            # rospy.loginfo("State is %s and prev state is %s" %(self.curr_ltl_state, self.prev_ltl_state))
            rate.sleep()    

#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_a1',anonymous=False)
    try:
        ltl_a1 = LTLControllerA1()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL a1 node: %s" %(e))
        sys.exit(0)
    
