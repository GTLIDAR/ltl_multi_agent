#!/usr/bin/env python
import roslib
import numpy
import rospy
import sys
import importlib
import yaml
import time

from copy import deepcopy

import std_msgs

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.ltl_planner_multi_robot_exp import LTLPlanner_MultiRobot_Exp

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg

# Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_msgs.srv import TaskPlanning, TaskPlanningResponse
from networkx.drawing.nx_agraph import to_agraph

# Import dynamic reconfigure components for dynamic parameters (see dynamic_reconfigure and dynamic_params package)
from dynamic_reconfigure.server import Server as DRServer
from ltl_automaton_planner.cfg import LTLAutomatonDPConfig
from ltl_automation_a1.srv import LTLTrace


def show_automaton(automaton_graph):
    # pos=nx.circular_layout(automaton_graph)
    # nx.draw(automaton_graph, pos)
    # nx.draw_networkx_labels(automaton_graph, pos)
    # edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    # nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    # plt.show()
    automaton_graph.graph['edge'] = {'arrowsize': '0.6', 'splines': 'curved'}
    A = to_agraph(automaton_graph)
    A.layout('dot')
    A.draw('team.png')
    plt.show()
    return

class MultiRobot_Planner_Exp(object):
    def __init__(self):
        # init parameters, automaton, etc...
        self.init_params()

        self.build_automaton()

        self.setup_pub_sub()

        # Output plan and first command of plan
        # self.publish_possible_states()
        self.publish_plan_initial()
        # self.plan_pub.publish(self.ltl_planner.next_move)


    def init_params(self):
        #Get parameters from parameter server
        self.agent_name_mobile_1 = rospy.get_param('agent_name_mobile_1', "agent_1")
        self.agent_name_mobile_2 = rospy.get_param('agent_name_mobile_2', "agent_2")
        self.agent_name_dog_1 = rospy.get_param('agent_name_dog_1', "dog_1")
        self.initial_beta = rospy.get_param('initial_beta', 1000)
        self.gamma = rospy.get_param('gamma', 10)


        # LTL task
        #----------
        # Get LTL hard task and raise error if don't exist
        if (rospy.has_param('hard_task')):
            self.hard_task = rospy.get_param('hard_task')
        else:
            raise InitError("Cannot initialize LTL planner, no hard_task defined")
        # Get LTL soft task
        self.soft_task = rospy.get_param('soft_task', "")


        # Transition system
        #-------------------
        # Get TS from param
        transition_system_mobile_1_textfile = rospy.get_param('transition_system_mobile_1_textfile')
        self.transition_system_mobile_1 = import_ts_from_file(transition_system_mobile_1_textfile)

        transition_system_mobile_2_textfile = rospy.get_param('transition_system_mobile_2_textfile')
        self.transition_system_mobile_2 = import_ts_from_file(transition_system_mobile_2_textfile)

        transition_system_quadruped_textfile = rospy.get_param('transition_system_quadruped_textfile')
        self.transition_system_quadruped = import_ts_from_file(transition_system_quadruped_textfile)
        #print(self.transition_system)

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = rospy.get_param('~initial_ts_state_from_agent', False)

        #If initial TS states is from agent, wait from agent state callback
        # if self.initial_ts_state_from_agent:
        #     self.initial_state_ts_dict = None
        #     rospy.loginfo("LTL planner: waiting for initial TS state from agent to initialize")
        #     while not self.initial_state_ts_dict:
        #         self.initial_state_ts_dict = self.init_ts_state_from_agent(rospy.wait_for_message("ts_state", TransitionSystemStateStamped))
        # else:
        self.initial_state_ts_dict = None


        # Setup dynamic parameters (defined in dynamic_params/cfg/LTL_automaton_dynparam.cfg)
        self.replan_on_unplanned_move = True
        self.check_timestamp = True
        self.prev_received_timestamp_1 = rospy.Time()
        self.prev_received_timestamp_2 = rospy.Time()
        self.prev_received_timestamp_3 = rospy.Time()


    def init_ts_state_from_agent(self, msg=TransitionSystemStateStamped):
        initial_state_ts_dict_ = None

        # If message is conform (same number of state as number of state dimensions)
        if (len(msg.ts_state.states) == len(msg.ts_state.state_dimension_names)):
            # Create dictionnary with paired dimension_name/state_value
            initial_state_ts_dict_ = dict()
            for i in range(len(msg.ts_state.states)):
                initial_state_ts_dict_.update({msg.ts_state.state_dimension_names[i] : msg.ts_state.states[i]})

                # Else message is malformed, raise error
        else:
            rospy.logerr("LTL planner: received initial states don't match TS state models: "+str(len(msg.ts_state.states))+" initial states and "+str(len(msg.ts_state.state_dimension_names))+" state models")

        # Return initial state dictionnary
        return initial_state_ts_dict_


    def build_automaton(self):
        # Import state models from TS
        state_models_mobile_1 = state_models_from_ts(self.transition_system_mobile_1, self.initial_state_ts_dict)
        state_models_mobile_2 = state_models_from_ts(self.transition_system_mobile_2, self.initial_state_ts_dict)
        state_models_quadruped = state_models_from_ts(self.transition_system_quadruped, self.initial_state_ts_dict)

        # Here we take the product of each element of state_models to define the full TS
        self.robot_model_mobile_1 = TSModel(state_models_mobile_1)
        self.robot_model_mobile_2 = TSModel(state_models_mobile_2)
        self.robot_model_quadruped = TSModel(state_models_quadruped)
        self.ts_list = [self.robot_model_mobile_1, self.robot_model_quadruped, self.robot_model_mobile_2]

        self.ltl_planner_multi_robot = LTLPlanner_MultiRobot_Exp(self.ts_list, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner_multi_robot.task_allocate()
        # Get first value from set
        # self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        # self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        # show_automaton(self.robot_model)
        # show_automaton(self.ltl_planner_multi_robot.buchi)
        # show_automaton(self.ltl_planner.product)
        # show_automaton(self.ltl_planner_multi_robot.team)


    def setup_pub_sub(self):
        # Prefix plan publisher
        self.plan_pub_1 = rospy.Publisher('/action_plan_1', LTLPlan, latch=True, queue_size = 1)
        self.plan_pub_2 = rospy.Publisher('/action_plan_2', LTLPlan, latch=True, queue_size = 1)
        self.plan_pub_3 = rospy.Publisher('/action_plan_3', LTLPlan, latch=True, queue_size = 1)

        # Possible states publisher
        # self.possible_states_pub = rospy.Publisher('possible_ltl_states', LTLStateArray, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.trace_sub_1 = rospy.Subscriber('/dr_0/ltl_trace', LTLPlan, self.ts_trace_callback_1, queue_size=1)
        self.trace_sub_2 = rospy.Subscriber('/a1_gazebo/ltl_trace', LTLPlan, self.ts_trace_callback_2, queue_size=1)
        self.trace_sub_3 = rospy.Subscriber('/wassi_0/ltl_trace', LTLPlan, self.ts_trace_callback_3, queue_size=1)

        # Subscribe to the replanning status
        self.replan_sub_1 = rospy.Subscriber('/dr_0/replanning_request', std_msgs.msg.Int8, self.ltl_replan_callback_1, queue_size=1)
        self.replan_sub_2 = rospy.Subscriber('/a1_gazebo/replanning_request', std_msgs.msg.Int8, self.ltl_replan_callback_2, queue_size=1)
        self.replan_sub_3 = rospy.Subscriber('/wassi_0/replanning_request', std_msgs.msg.Int8, self.ltl_replan_callback_3, queue_size=1)

    def ltl_replan_callback_1(self, msg):
        replan_status = msg.data
        if(replan_status == 0):
            rospy.logerr('LTL planner: replanning ERROR')

        if(replan_status == 1):
            rospy.logwarn('LTL planner: received replanning Level 1; handling malfunction from agent 1')
            start = time.time()
            #Replan
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 0
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            #TODO: Add ros service for requesting the synchronization
            service_1 = rospy.ServiceProxy('/dr_0/synchronization_service', LTLTrace)
            service_1(request=1)
            service_2 = rospy.ServiceProxy('/a1_gazebo/synchronization_service', LTLTrace)
            service_2(request=1)
            service_3 = rospy.ServiceProxy('/wassi_0/synchronization_service', LTLTrace)
            service_3(request=1)

            while (len(self.ltl_planner_multi_robot.trace_dic[0]) == 0) or \
                    (len(self.ltl_planner_multi_robot.trace_dic[1]) == 0) or \
                    (len(self.ltl_planner_multi_robot.trace_dic[2]) == 0):
                rospy.logwarn('Waiting for the trace callback from all agents')

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 1 done within %.2fs' %(time.time()-start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

        if(replan_status == 2):
            rospy.logwarn('LTL planner: received replanning Level 2: handling abrupt state change from agent 1')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 0
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[0]) == 0:
                rospy.logwarn('Waiting for the trace callback from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_2()
            if success:
                if level_flag=="Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 2 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[0] = list()

                if level_flag=="Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 2 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None

        if(replan_status == 3):
            rospy.logwarn('LTL planner: received replanning Level 3: handling transition system change from agent 1')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 0
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[0]) == 0:
                rospy.logwarn('Waiting for the trace and Updated TS callbacks from agent 1')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_3()
            if success:
                if level_flag=="Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 3 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[0] = list()

                if level_flag=="Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 3 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}


    def ltl_replan_callback_2(self, msg):
        replan_status = msg.data
        if(replan_status == 0):
            rospy.logerr('LTL planner: replanning ERROR')

        if(replan_status == 1):
            rospy.logwarn('LTL planner: received replanning Level 1; handling malfunction from agent 2')
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 1
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")
            #Replan
            start = time.time()
            #TODO: Add ros service for requesting the synchronization
            service_1 = rospy.ServiceProxy('/dr_0/synchronization_service', LTLTrace)
            service_1(request=1)
            service_2 = rospy.ServiceProxy('/a1_gazebo/synchronization_service', LTLTrace)
            service_2(request=1)
            service_3 = rospy.ServiceProxy('/wassi_0/synchronization_service', LTLTrace)
            service_3(request=1)


            while (len(self.ltl_planner_multi_robot.trace_dic[0]) == 0) or \
                    (len(self.ltl_planner_multi_robot.trace_dic[1]) == 0) or \
                    (len(self.ltl_planner_multi_robot.trace_dic[2]) == 0):
                rospy.logwarn('Waiting for the trace callback from all agents')

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 1 done within %.2fs' %(time.time()-start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

        if(replan_status == 2):
            rospy.logwarn('LTL planner: received replanning Level 2: handling abrupt state change from agent 2')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 1
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[1]) == 0:
                rospy.logwarn('Waiting for the trace callback from agent 2')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_2()
            if success:
                if level_flag == "Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 2 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[1] = list()

                if level_flag == "Global":
                    self.publish_plan_initial()
                    self.ltl_planner_multi_robot.trace_dic = {}
                    rospy.logwarn('Replanning level 2 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None

        if(replan_status == 3):
            rospy.logwarn('LTL planner: received replanning Level 3: handling transition system change from agent 2')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 1
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[1]) == 0:
                rospy.logwarn('Waiting for the trace and Updated TS callbacks from agent 2')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_3()
            if success:
                if level_flag=="Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 3 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[1] = list()

                if level_flag=="Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 3 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None


    def ltl_replan_callback_3(self, msg):
        replan_status = msg.data
        if(replan_status == 0):
            rospy.logerr('LTL planner: replanning ERROR')

        if(replan_status == 1):
            rospy.logwarn('LTL planner: received replanning Level 1; handling malfunction from agent 3')
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 2
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")
            #Replan
            start = time.time()
            #TODO: Add ros service for requesting the synchronization
            service_1 = rospy.ServiceProxy('/dr_0/synchronization_service', LTLTrace)
            service_1(request=1)
            service_2 = rospy.ServiceProxy('/a1_gazebo/synchronization_service', LTLTrace)
            service_2(request=1)
            service_3 = rospy.ServiceProxy('/wassi_0/synchronization_service', LTLTrace)
            service_3(request=1)


            while (len(self.ltl_planner_multi_robot.trace_dic[0]) == 0) or \
                    (len(self.ltl_planner_multi_robot.trace_dic[1]) == 0) or \
                    (len(self.ltl_planner_multi_robot.trace_dic[2]) == 0):
                rospy.logwarn('Waiting for the trace callback from all agents')

            if self.ltl_planner_multi_robot.replan_level_1():
                self.publish_plan_initial()
                rospy.logwarn('Replanning level 1 done within %.2fs' %(time.time()-start))
                self.ltl_planner_multi_robot.trace_dic = {}
                self.ltl_planner_multi_robot.trace_dic[0] = list()
                self.ltl_planner_multi_robot.trace_dic[1] = list()
                self.ltl_planner_multi_robot.trace_dic[2] = list()
                self.ltl_planner_multi_robot.local_replan_rname = None
                self.ltl_planner_multi_robot.update_info = {}

        if(replan_status == 2):
            rospy.logwarn('LTL planner: received replanning Level 2: handling abrupt state change from agent 3')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 2
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[2]) == 0:
                rospy.logwarn('Waiting for the trace callback from agent 3')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_2()
            if success:
                if level_flag == "Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 2 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                if level_flag == "Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 2 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None

        if(replan_status == 3):
            rospy.logwarn('LTL planner: received replanning Level 3: handling transition system change from agent 3')
            # Replan
            start = time.time()
            # if self.ltl_planner_multi_robot.local_replan_rname is not None:
            self.ltl_planner_multi_robot.local_replan_rname = 2
            rospy.loginfo("LTL planner: local replan rname is: %d" %self.ltl_planner_multi_robot.local_replan_rname)
            # else:
            #     rospy.logerr("LTL planner: local replan rname is not empty")

            while len(self.ltl_planner_multi_robot.trace_dic[2]) == 0:
                rospy.logwarn('Waiting for the trace and Updated TS callbacks from agent 3')

            level_flag, success = self.ltl_planner_multi_robot.replan_level_3()
            if success:
                if level_flag=="Local":
                    self.publish_local(self.ltl_planner_multi_robot.local_replan_rname)
                    rospy.logwarn('Replanning level 3 local done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                if level_flag=="Global":
                    self.publish_plan_initial()
                    rospy.logwarn('Replanning level 3 global done within %.2fs' %(time.time()-start))
                    self.ltl_planner_multi_robot.trace_dic = {}
                    self.ltl_planner_multi_robot.trace_dic[0] = list()
                    self.ltl_planner_multi_robot.trace_dic[1] = list()
                    self.ltl_planner_multi_robot.trace_dic[2] = list()

                self.ltl_planner_multi_robot.local_replan_rname = None


    def ts_trace_callback_1(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_1.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_1 = deepcopy(msg.header.stamp)
            self.ltl_planner_multi_robot.trace_dic[0] = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 0

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                #-------------------------
                # Check if state is in TS
                #-------------------------
                if (state in self.robot_model_mobile_1.nodes()):

                    # Update trace for robot 1
                    self.ltl_planner_multi_robot.trace_dic[0].append(state)

                #--------------------------------------------
                # If state not part of the transition system
                #--------------------------------------------
                else:
                    #ERROR: unknown state (not part of TS)
                    rospy.logwarn('State is not in TS plan!')
        else:
            rospy.logwarn("LTL planner: not updating with received trace, timestamp identical to previously received message timestamp at time %f" %self.prev_received_timestamp_1.to_sec())




    def ts_trace_callback_2(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_2.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_2 = deepcopy(msg.header.stamp)
            self.ltl_planner_multi_robot.trace_dic[1] = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 1

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                #-------------------------
                # Check if state is in TS
                #-------------------------
                if (state in self.robot_model_quadruped.nodes()):

                    # Update trace for robot 2
                    self.ltl_planner_multi_robot.trace_dic[1].append(state)

                #--------------------------------------------
                # If state not part of the transition system
                #--------------------------------------------
                else:
                    #ERROR: unknown state (not part of TS)
                    rospy.logwarn('State is not in TS plan!')
        else:
            rospy.logwarn("LTL planner: not updating with received trace, timestamp identical to previously received message timestamp at time %f" %self.prev_received_timestamp_2.to_sec())


    def ts_trace_callback_3(self, msg=LTLPlan()):
        # Extract TS state from message
        if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp_3.to_sec())):
            # Update previously received timestamp
            self.prev_received_timestamp_3 = deepcopy(msg.header.stamp)
            self.ltl_planner_multi_robot.trace_dic[2] = list()
            # self.ltl_planner_multi_robot.local_replan_rname = 2

            for state_msg in msg.ts_state_sequence:
                state = handle_ts_state_msg(state_msg)

                #-------------------------
                # Check if state is in TS
                #-------------------------
                if (state in self.robot_model_mobile_2.nodes()):

                    # Update trace for robot 2
                    self.ltl_planner_multi_robot.trace_dic[2].append(state)

                #--------------------------------------------
                # If state not part of the transition system
                #--------------------------------------------
                else:
                    #ERROR: unknown state (not part of TS)
                    rospy.logwarn('State is not in TS plan!')
        else:
            rospy.logwarn("LTL planner: not updating with received trace, timestamp identical to previously received message timestamp at time %f" %self.prev_received_timestamp_3.to_sec())



    #----------------------------------------------
    # Publish prefix and suffix plans from planner
    #----------------------------------------------
    def publish_plan_initial(self):
        # If plan exists
        if not (self.ltl_planner_multi_robot.plans == None):
            # Prefix plan
            #-------------
            plan_1_msg = LTLPlan()
            plan_1_msg.header.stamp = rospy.Time.now()
            plan_1_status = False
            plan_2_msg = LTLPlan()
            plan_2_msg.header.stamp = rospy.Time.now()
            plan_2_status = False
            plan_3_msg = LTLPlan()
            plan_3_msg.header.stamp = rospy.Time.now()
            plan_3_status = False

            # plan_1_msg.action_sequence = self.ltl_planner.run.pre_plan
            # Go through all TS state in plan and add it as TransitionSystemState message
            for r_idx, act_seq in self.ltl_planner_multi_robot.plans.action_sequence.items():
                if r_idx==0: # and len(act_seq) != 0:
                    plan_1_status = True
                    plan_1_msg.action_sequence = act_seq
                if r_idx==1: # and len(act_seq) != 0:
                    plan_2_status = True
                    plan_2_msg.action_sequence = act_seq
                if r_idx==2: # and len(act_seq) != 0:
                    plan_3_status = True
                    plan_3_msg.action_sequence = act_seq

            for r_idx, stat_seq in self.ltl_planner_multi_robot.plans.ts_state_sequence.items():
                if r_idx==0 and plan_1_status:
                    for ts_state in stat_seq:
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_multi_robot.pro_list_initial[r_idx].graph['ts'].graph['ts_state_format'] for item in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        plan_1_msg.ts_state_sequence.append(ts_state_msg)

                    # Publish
                    self.plan_pub_1.publish(plan_1_msg)

                if r_idx==1 and plan_2_status:
                    for ts_state in stat_seq:
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_multi_robot.pro_list_initial[r_idx].graph['ts'].graph['ts_state_format'] for item in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        plan_2_msg.ts_state_sequence.append(ts_state_msg)

                    # Publish
                    self.plan_pub_2.publish(plan_2_msg)

                if r_idx==2 and plan_3_status:
                    for ts_state in stat_seq:
                        ts_state_msg = TransitionSystemState()
                        ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_multi_robot.pro_list_initial[r_idx].graph['ts'].graph['ts_state_format'] for item in sublist]
                        # If TS state is more than 1 dimension (is a tuple)
                        if type(ts_state) is tuple:
                            ts_state_msg.states = list(ts_state)
                        # Else state is a single string
                        else:
                            ts_state_msg.states = [ts_state]
                        # Add to plan TS state sequence
                        plan_3_msg.ts_state_sequence.append(ts_state_msg)

                    # Publish
                    self.plan_pub_3.publish(plan_3_msg)


    def publish_local(self, rname):
        # If plan exists
        if self.ltl_planner_multi_robot.local_plan is not None:
            # Prefix plan
            #-------------
            plan_local_msg = LTLPlan()
            plan_local_msg.header.stamp = rospy.Time.now()
            plan_local_status = False

            plan_local_msg.action_sequence = self.ltl_planner_multi_robot.local_plan.action_sequence

            for ts_state in self.ltl_planner_multi_robot.local_plan.ts_state_sequence:
                ts_state_msg = TransitionSystemState()
                ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner_multi_robot.pro_list_initial[rname].graph['ts'].graph['ts_state_format'] for item in sublist]
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]
                # Add to plan TS state sequence
                plan_local_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            if rname == 0:
                self.plan_pub_1.publish(plan_local_msg)
            elif rname == 1:
                self.plan_pub_2.publish(plan_local_msg)
            elif rname == 2:
                self.plan_pub_3.publish(plan_local_msg)
            else:
                rospy.logerr("LTL Planner: rname in local publisher doesn't match!")
                return False

        return True


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_planner_multi_robot_exp', anonymous=False)
    try:
        multi_robot_ltl_planner_node = MultiRobot_Planner_Exp()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Planner: "+str(e))
        rospy.logerr("LTL Planner: shutting down...")
