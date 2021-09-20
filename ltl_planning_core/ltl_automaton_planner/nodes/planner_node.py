#!/usr/bin/env python
import roslib
import numpy
import rospy
import sys
import importlib
import yaml

from copy import deepcopy

import std_msgs

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg

# Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_msgs.srv import TaskPlanning, TaskPlanningResponse

# Import dynamic reconfigure components for dynamic parameters (see dynamic_reconfigure and dynamic_params package)
from dynamic_reconfigure.server import Server as DRServer
from ltl_automaton_planner.cfg import LTLAutomatonDPConfig


def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    plt.show()
    return

class MainPlanner(object):
    def __init__(self):
        # init parameters, automaton, etc...
        self.init_params()

        self.build_automaton()

        self.setup_pub_sub()

        self.setup_plugins()

        # Output plan and first command of plan
        self.publish_possible_states()
        self.publish_plan()
        self.plan_pub.publish(self.ltl_planner.next_move)
       

    def init_params(self):
        #Get parameters from parameter server
        self.agent_name = rospy.get_param('agent_name', "agent")
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
        transition_system_textfile = rospy.get_param('transition_system_textfile')
        self.transition_system = import_ts_from_file(transition_system_textfile)
        #print(self.transition_system)

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = rospy.get_param('~initial_ts_state_from_agent', False)

        #If initial TS states is from agent, wait from agent state callback
        if self.initial_ts_state_from_agent:
            self.initial_state_ts_dict = None
            rospy.loginfo("LTL planner: waiting for initial TS state from agent to initialize")
            while not self.initial_state_ts_dict:
                self.initial_state_ts_dict = self.init_ts_state_from_agent(rospy.wait_for_message("ts_state", TransitionSystemStateStamped))
        else:
            self.initial_state_ts_dict = None


        # Setup dynamic parameters (defined in dynamic_params/cfg/LTL_automaton_dynparam.cfg)
        self.replan_on_unplanned_move = True
        self.check_timestamp = True
        self.prev_received_timestamp = rospy.Time()
        self.dynparam_srv = DRServer(LTLAutomatonDPConfig, self.dynparam_callback)
    
    #---------------------------------------------
    # Load plugins from a given plugin dictionary
    #---------------------------------------------
    def load_and_init_plugins(self, plugin_dict):
    # ----
    # Format for plugin param is:
    #   plugin/<plugin-name>:
    #       path: <package-path>
    #       args: <additional-argument-dictionary>
    # ----
        self.plugins = {}
        for plugin in plugin_dict:
            # Import plugin module
            try:
                # Import module to a plugin dict
                plugin_module = importlib.import_module(plugin_dict[plugin]["path"])
            except ImportError:
                # Error log message
                rospy.logerr("LTL planner: Import error on loading plugin %s at %s" % (str(plugin), plugin_dict[plugin]["path"]))
                # Go to next plugin
                break

            # Get plugin class from imported module
            plugin_class = getattr(plugin_module, str(plugin))
            # Create plugin object from class using argument dictionary from parameters
            self.plugins.update({plugin: plugin_class(self.ltl_planner, plugin_dict[plugin]['args'])})
            rospy.loginfo("LTL planner: using plugin %s" % str(plugin))

        # Init plugins
        for plugin in self.plugins:
            self.plugins[plugin].init()

     
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
        state_models = state_models_from_ts(self.transition_system, self.initial_state_ts_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = TSModel(state_models)
        self.ltl_planner = LTLPlanner(self.robot_model, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner.optimal()
        # Get first value from set
        self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        # show_automaton(self.robot_model)
        # show_automaton(self.ltl_planner.product.graph['buchi'])
        # show_automaton(self.ltl_planner.product)


    def setup_pub_sub(self):
        # Prefix plan publisher
        self.prefix_plan_pub = rospy.Publisher('prefix_plan', LTLPlan, latch=True, queue_size = 1)

        # Suffix plan publisher
        self.suffix_plan_pub = rospy.Publisher('suffix_plan', LTLPlan, latch=True, queue_size = 1)

        # Possible states publisher
        self.possible_states_pub = rospy.Publisher('possible_ltl_states', LTLStateArray, latch=True, queue_size=1)

        # Initialize subscriber to provide current state of robot
        self.state_sub = rospy.Subscriber('ts_state', TransitionSystemStateStamped, self.ltl_state_callback, queue_size=1) 

        # Initialize publisher to send plan commands
        self.plan_pub = rospy.Publisher('next_move_cmd', std_msgs.msg.String, queue_size=1, latch=True)

        # Initialize task replanning service
        self.trap_srv = rospy.Service('replanning', TaskPlanning, self.task_replanning_callback)

        # Subscribe to the replanning status
        self.replan_sub = rospy.Subscriber('replanning_request', std_msgs.msg.Int8, self.ltl_replan_callback, queue_size=1)


    def setup_plugins(self):
        # Get plugin dictionnary from parameters
        plugin_dict = rospy.get_param('~plugin', {})
        # If plugins are specified, try to load them
        self.load_and_init_plugins(plugin_dict)

        # Setup plugin subscribers and publisers
        for plugin in self.plugins:
            self.plugins[plugin].set_sub_and_pub()


    def dynparam_callback(self, config, level):
        self.replan_on_unplanned_move = config['replan_on_unplanned_move']
        self.check_timestamp = config['check_timestamp']
        return config


    def task_replanning_callback(self, task_planning_req):
        # Extract task specification from request
        hard_task = task_planning_req.hard_task
        soft_task = task_planning_req.soft_task
        # Create response message
        rsp = TaskPlanningResponse()
        # Replan and return success status
        rsp.success = self.ltl_planner.replan_task(hard_task, soft_task, self.ltl_planner.curr_ts_state)
        # If successful, change parameters
        if rsp.success:
            rospy.set_param('hard_task', hard_task)
            rospy.set_param('soft_task', soft_task)
        # Return response
        return rsp


    def ltl_replan_callback(self, msg):
        replan_status = msg.data
        rospy.logerr('LTL planner: replanning debug')
        if(replan_status == 0):
            rospy.logerr('LTL planner: replanning ERROR')

        if(replan_status == 2):
            rospy.logwarn('LTL planner: received replanning Level 2')
            # Replan
            # self.ltl_planner.replan_from_ts_state(state)
            # self.publish_plan()

        if(replan_status == 3):
            rospy.logwarn('LTL planner: received replanning Level 3')




    def ltl_state_callback(self, msg=TransitionSystemStateStamped()):
        # Extract TS state from message
        state = handle_ts_state_msg(msg.ts_state)
        
        #-------------------------
        # Check if state is in TS
        #-------------------------
        if (state in self.robot_model.nodes()):

            # If timestamp check is enabled, check the timestamp
            if not (self.check_timestamp and (msg.header.stamp.to_sec() == self.prev_received_timestamp.to_sec())):
                # Update previously received timestamp
                self.prev_received_timestamp = deepcopy(msg.header.stamp)

                # Update current state
                self.ltl_planner.curr_ts_state = state

                #-----------------------------------------------------------------------
                # Try update possible state and if error (forbidden transition), replan
                #-----------------------------------------------------------------------
                if not self.ltl_planner.update_possible_states(state):
                    rospy.logerr('Can not update possible states - forbidden transition, replanning...')

                    # Replan
                    # self.ltl_planner.replan_from_ts_state(state)
                    # self.publish_plan()
                    
                    # Publish next move
                    rospy.logwarn('LTL planner: error in possible states, replanning done and publishing next move')
                    self.plan_pub.publish(self.ltl_planner.next_move)

                    return

                # Publish possible states
                self.publish_possible_states()

                #--------------------------
                # Manage next move in plan
                #--------------------------
                # If state is next state in plan, find next_move and output
                if self.is_next_state_in_plan(state):
                    self.ltl_planner.find_next_move()

                    # Publish next move
                    rospy.loginfo('LTL planner: Publishing next move')
                    self.plan_pub.publish(self.ltl_planner.next_move)

                # If state is not the next one in plan replan 
                elif self.replan_on_unplanned_move:
                    rospy.logwarn('LTL planner: Received state is not the next one in the plan, replanning and publishing next move')
                    # Replan with state as initial
                    # self.ltl_planner.replan_from_ts_state(state)
                    # self.publish_plan()

                    # Publish next move
                    self.plan_pub.publish(self.ltl_planner.next_move)

                #-------------
                # Run plugins
                #-------------
                for plugin in self.plugins:
                    self.plugins[plugin].run_at_ts_update(state)

            # If timestamp is indentical to previoulsy received message and parameters "check_timestamp" is true
            else:
                rospy.logwarn("LTL planner: not updating with received TS state %s, timestamp identical to previously received message timestamp at time %f" % (str(state), self.prev_received_timestamp.to_sec()))

        #--------------------------------------------
        # If state not part of the transition system
        #--------------------------------------------
        else:
            #ERROR: unknown state (not part of TS)
            self.plan_pub.publish('None')
            rospy.logwarn('State is not in TS plan!')

    #-----------------------------------------------------
    # Check if given TS state is the next one in the plan
    #-----------------------------------------------------
    def is_next_state_in_plan(self, ts_state):
        # Check if plan is in prefix phase
        if self.ltl_planner.segment == 'line':
            # Check if state is the next state of the plan
            if ts_state == self.ltl_planner.run.line[self.ltl_planner.index+1]:
                return True
        # Check if plan is in suffix phase
        elif self.ltl_planner.segment == 'loop':
            # Check if state is the next state of the plan
            if ts_state == self.ltl_planner.run.loop[self.ltl_planner.index+1]:
                return True
        return False

    #----------------------------------------------
    # Publish prefix and suffix plans from planner
    #----------------------------------------------
    def publish_plan(self):
        # If plan exists
        if not (self.ltl_planner.run == None):
            # Prefix plan
            #-------------
            prefix_plan_msg = LTLPlan()
            prefix_plan_msg.header.stamp = rospy.Time.now()
            prefix_plan_msg.action_sequence = self.ltl_planner.run.pre_plan
            # Go through all TS state in plan and add it as TransitionSystemState message
            for ts_state in self.ltl_planner.run.line:
                ts_state_msg = TransitionSystemState()
                ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner.product.graph['ts'].graph['ts_state_format'] for item in sublist]
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]
                # Add to plan TS state sequence
                prefix_plan_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            self.prefix_plan_pub.publish(prefix_plan_msg)

            # Suffix plan
            #-------------
            suffix_plan_msg = LTLPlan()
            suffix_plan_msg.header.stamp = rospy.Time.now()
            suffix_plan_msg.action_sequence = self.ltl_planner.run.suf_plan
            # Go through all TS state in plan and add it as TransitionSystemState message
            for ts_state in self.ltl_planner.run.loop:
                ts_state_msg = TransitionSystemState()
                ts_state_msg.state_dimension_names = [item for sublist in self.ltl_planner.product.graph['ts'].graph['ts_state_format'] for item in sublist]
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]

                # Add to plan TS state sequence
                suffix_plan_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            self.suffix_plan_pub.publish(suffix_plan_msg)

    #-------------------------
    # Publish possible states
    #-------------------------
    def publish_possible_states(self):
        # Create message
        possible_states_msg = LTLStateArray()
        # For all possible state, add to the message list
        for ltl_state in self.ltl_planner.product.possible_states:
            ltl_state_msg = LTLState()
            # If TS state is more than 1 dimension (is a tuple)
            if type(ltl_state[0]) is tuple:
                ltl_state_msg.ts_state.states = list(ltl_state[0])
            # Else state is a single string
            else:
                ltl_state_msg.ts_state.states = [ltl_state[0]]

            ltl_state_msg.ts_state.state_dimension_names = self.ltl_planner.product.graph['ts'].graph['ts_state_format']
            ltl_state_msg.buchi_state = str(ltl_state[1])
            possible_states_msg.ltl_states.append(ltl_state_msg)

        # Publish
        self.possible_states_pub.publish(possible_states_msg)


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_planner', anonymous=False)
    try:
        ltl_planner_node = MainPlanner()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Planner: "+str(e))
        rospy.logerr("LTL Planner: shutting down...")
