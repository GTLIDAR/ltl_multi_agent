# -*- coding: utf-8 -*-
import rospy
from ltl_automaton_planner.ltl_tools.buchi import mission_to_buchi
from ltl_automaton_planner.ltl_tools.product import ProdAut
#from ts import distance, reach_waypoint
from ltl_automaton_planner.ltl_tools.discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history
import networkx as nx

###########################################################
# Construct a general class for handling the TS and Buchi
# automation given LTL specification; all the graph search
# happen here
# Input is TS, LTL tasks and other user inputs
###########################################################

class LTLPlanner(object):
    def __init__(self, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        self.hard_spec = hard_spec
        self.soft_spec = soft_spec
        self.ts = ts

        #Empty product
        self.product = None

        self.Time = 0
        self.curr_ts_state = None
        self.trace = [] # record the regions been visited
        self.traj = [] # record the full trajectory
        self.opt_log = [] 
        # record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
        self.com_log = []
        # record [(time, no_messages)]
        self.beta = beta                    # importance of taking soft task into account
        self.gamma = gamma                  # cost ratio between prefix and suffix


    def optimal(self, style='static'):
        rospy.loginfo("LTL Planner: --- Planning in progress ("+style+") ---")
        rospy.loginfo("LTL Planner: Hard task is: "+str(self.hard_spec))
        rospy.loginfo("LTL Planner: Soft task is: "+str(self.soft_spec))

        if style == 'static':
            # full graph construction
            self.product = ProdAut(self.ts, mission_to_buchi(self.hard_spec, self.soft_spec), self.beta)
            self.product.graph['ts'].build_full()
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
        elif style == 'ready':
            if self.product:
                self.product.build_full()
                self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
            else:
                rospy.logerr("LTL Planner: \"ready\" planning was requested but product graph was never built, aborting...")
                return False
        elif style == 'on-the-fly-initial':
            if self.product:
                # on-the-fly construction, only sets the initial states and replan
                self.product.build_initial()
                self.product.build_accept() 
                self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
            else:
                rospy.logerr("LTL Planner: \"on-the-fly-initial\" planning was requested but product graph was never built, aborting...")
                return False
        elif style == 'on-the-fly-task':
            if self.product:
                # on-the-fly task construction, only reset buchi, initial states and replan
                self.product.graph['buchi'] = mission_to_buchi(self.hard_spec, self.soft_spec)
                self.product.build_full()
                self.run, plantime = dijkstra_plan_networkX(self.product, self.gamma)
            else:
                rospy.logerr("LTL Planner: \"on-the-fly-task\" planning was requested but product graph was never built, aborting...")
                return False

        if self.run == None:
            rospy.logerr("LTL Planner: No valid plan has been found! Check you FTS or task")
            return False

        rospy.loginfo("LTL Planner: --- Planning successful! ---")
        rospy.logdebug("Prefix states: "+str([n for n in self.run.line]))
        rospy.logdebug("Suffix states: "+str([n for n in self.run.loop]))

        self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
        self.last_time = self.Time
        self.acc_change = 0
        self.index = 0
        self.segment = 'line'
        
        # If prefix exists, init next move with prefix first action
        if self.run.pre_plan:
            self.next_move = self.run.pre_plan[self.index]
        # If prefix is empty, jump to suffix for init next move
        else:
            self.next_move = self.run.suf_plan[self.index]
            self.segment = 'loop'
        return True

    #-----------------------------
    #   Update current possible   
    # states using given TS state 
    #-----------------------------
    def update_possible_states(self, ts_node):
        # Get possible states
        self.product.possible_states = self.product.get_possible_states(ts_node)

        if self.start_suffix():
            self.product.possible_states = self.intersect_accept(self.product.possible_states, ts_node)

        # If possible states set in not empty, return true
        if self.product.possible_states:
            return True
        # If no states in possible states set, return false
        else:
            return False

    def intersect_accept(self, possible_states, reach_ts):
        accept_set = self.product.graph['accept']
        inter_set = set([s for s in accept_set if s[0] == reach_ts])
        return inter_set


    def start_suffix(self):
        if ((self.segment == 'loop') and (self.index == 0)):
            return True
        else:
            return False

    def find_next_move(self):
        # Check if plan is in 'line' i.e. prefix or 'loop' i.e. suffix

        # if index is not the last of the pre_plan...
        if self.segment == 'line' and self.index < len(self.run.pre_plan)-1:

            # Add the node that has been visited to trace
            self.trace.append(self.run.line[self.index])

            # Increment index counter
            self.index += 1

            # Extract next move from pre_plan
            self.next_move = self.run.pre_plan[self.index]

        # If index is the last of the pre-plan or equivalently if the pre_plan is short...
        elif (self.segment == 'line') and ((self.index == len(self.run.pre_plan)-1) or (len(self.run.pre_plan) <= 1)):

            # Add the node that has been visited to trace
            self.trace.append(self.run.line[self.index])

            # Reset index for the suffix loop
            self.index = 0

            # Change the segment type to loop
            self.segment = 'loop'

            # Extract first move of suffix plan
            self.next_move = self.run.suf_plan[self.index]

        # If index is not the last of the suffix plan or equivalently the suf_plan is short...
        elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-1:

            # Add the node that has been visited to trace
            self.trace.append(self.run.loop[self.index])

            # Increment the index
            self.index += 1

            # Extract next move from suffix plan
            self.next_move = self.run.suf_plan[self.index]

        # If index is the last element of the suf_plan or equivalently the suf_plan is short....
        elif (self.segment == 'loop') and ((self.index == len(self.run.suf_plan)-1) or (len(self.run.suf_plan) <= 1)):

            # Add the node that has been visited to trace
            self.trace.append(self.run.loop[self.index])

            # Resent index 
            self.index = 0

            # Extract next move from suffix plan
            self.next_move = self.run.suf_plan[self.index]
        return self.next_move

    #--------------------------------------
    # Given a new initial TS state, replan
    #--------------------------------------
    def replan_from_ts_state(self, ts_state):
        # Set new initial state in TS
        self.product.graph['ts'].set_initial(ts_state)
        # Use on-the-fly to only rebuild the initial product node
        return self.optimal(style="on-the-fly-initial")

    #--------------------------
    # Given a new task, replan
    #--------------------------
    def replan_task(self, hard_spec, soft_spec, initial_ts_state=None):
        # If initial state provided, modify initial
        if initial_ts_state:
            # Set new initial state in TS
            self.product.graph['ts'].set_initial(initial_ts_state)
            self.product.build_initial()
        # Set new task
        self.hard_spec = hard_spec
        self.soft_spec = soft_spec
        # Use on-the-fly to only rebuild buchi and the initial product node
        return self.optimal(style="on-the-fly-task")


    def replan(self):
        '''Create new system plan based on previous history'''
        new_run = improve_plan_given_history(self.product, self.trace)

        print('new_run = ' + str(new_run))
        print('------------------------------')
        print('the prefix of plan **states**:')
        print([n for n in new_run.line])
        print('the suffix of plan **states**:')
        print([n for n in new_run.loop])

        if (new_run) and (new_run.pre_plan !=self.run.pre_plan[self.index:-1]):
            self.run = new_run
            self.index = 1
            self.segment = 'line'
            self.next_move = self.run.pre_plan[self.index]
            print('Plan adapted!')
