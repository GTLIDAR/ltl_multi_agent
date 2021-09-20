import rospy
from ltl_automaton_planner.ltl_tools.team import TeamModel
from ltl_automaton_planner.ltl_tools.product import ProdAut
from ltl_automaton_planner.ltl_tools.buchi import mission_to_buchi
from ltl_automaton_planner.ltl_tools.decomposition_set import get_decomposition_set
from ltl_automaton_planner.ltl_tools.graph_search_team import compute_team_plans, compute_local_plan, find_reusable_plan
from ltl_automation_a1.srv import LTLTrace

class LTLPlanner_MultiRobot_Exp(object):
    def __init__(self, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        self.hard_spec = hard_spec
        self.soft_spec = soft_spec
        if ts:
            self.ts = ts
        else:
            rospy.logerr("TS input ERROR")

        self.pro_list_initial = []
        self.team = None
        self.buchi = None
        self.decomposition_set = set()
        self.trace_dic = {} # record the regions been visited
        self.trace_dic[0] = list()
        self.trace_dic[1] = list()
        self.trace_dic[2] = list()
        self.traj = [] # record the full trajectory
        self.ts_info = None
        self.local_replan_rname = None
        self.update_info = {}

        self.beta = beta                    # importance of taking soft task into account
        self.gamma = gamma                  # cost ratio between prefix and suffix
        self.build_product_list()

    def build_product_list(self):
        self.buchi = mission_to_buchi(self.hard_spec, self.soft_spec)
        self.decomposition_set = get_decomposition_set(self.buchi)
        for ts_0 in self.ts:
            product = ProdAut(ts_0, self.buchi)
            product.graph['ts'].build_full()
            product.build_full()
            self.pro_list_initial.append(product)

    def task_allocate(self, style='static'):
        rospy.loginfo("LTL Planner: --- Planning in progress ("+style+") ---")
        rospy.loginfo("LTL Planner: Hard task is: "+str(self.hard_spec))
        rospy.loginfo("LTL Planner: Soft task is: "+str(self.soft_spec))

        if style == 'static':
            self.team = TeamModel(self.pro_list_initial, self.decomposition_set)
            self.team.build_team()
            self.plans, plan_time = compute_team_plans(self.team)
            if self.plans is None:
                rospy.logerr("LTL Planner: No valid plan has been found!")
                return False

        if style == 'Global':
            if self.team and self.plans:
                self.team.revise_team(self.trace_dic, self.local_replan_rname, self.plans)
                self.plans, plan_time = compute_team_plans(self.team)
                if self.plans is None:
                    rospy.logerr("LTL Planner: No valid global reallocation plan has been found!")
                    return False
            else:
                rospy.logerr("LTL Planner: \"replanning_global: \" planning was requested but team model or previous plan was never built, aborting...")
                return False

        if style == 'Local_state_change':
            if self.team and self.plans:
                self.team.update_local_pa(self.trace_dic, self.local_replan_rname, self.plans)
                self.local_plan, self.local_plan_time = find_reusable_plan(self.team, self.local_replan_rname, self.plans)
                if self.local_plan is None:
                    rospy.logwarn("LTL Planner: Reusable path not found; Try local replanning")

                self.local_plan, self.local_plan_time = compute_local_plan(self.team, self.local_replan_rname)
                if self.local_plan is None:
                    rospy.logwarn("LTL Planner: No valid local plan has been found given state change! Try global option")
                    return False

            else:
                rospy.logerr("LTL Planner: \"replanning_local_state_change: \" planning was requested but team model or previous plan was never built, aborting...")
                return False

        if style == 'Local_ts_update':
            if self.team and self.plans:
                self.team.revise_local_pa(self.trace_dic, self.local_replan_rname, self.plans, self.update_info)
                self.local_plan, self.local_plan_time = compute_local_plan(self.team, self.local_replan_rname)
                if self.local_plan is None:
                    rospy.logwarn("LTL Planner: No valid local plan has been found given TS updates! Try global option")
                    return False
            else:
                rospy.logerr("LTL Planner: \"replanning_local_ts_change: \" planning was requested but team model or previous plan was never built, aborting...")
                return False

        return True

    def replan_level_1(self):
        #Directly do global reallocation because of malfunction
        #Remove the edges related to the malfunction agent
        self.update_info["added"] = set()
        self.update_info["relabel"] = set()
        self.update_info["deleted"] = self.team.find_deleted_malfunction(self.trace_dic, self.local_replan_rname)

        self.team.revise_local_pa(self.trace_dic, self.local_replan_rname, self.plans, self.update_info)
        return self.task_allocate(style="Global")

    def replan_level_2(self):
        #Try local replanning first
        if self.task_allocate(style="Local_state_change"):
            self.plans.state_sequence[self.local_replan_rname] = [(self.local_replan_rname, tt[0], tt[1]) for tt in self.local_plan.prefix]
            return "Local", True

        #TODO: Add ros service for requesting the synchronization
        service_1 = rospy.ServiceProxy('/dr_0/synchronization_service', LTLTrace)
        service_1(request=1)
        service_2 = rospy.ServiceProxy('/a1_gazebo/synchronization_service', LTLTrace)
        service_2(request=1)
        service_3 = rospy.ServiceProxy('/wassi_0/synchronization_service', LTLTrace)
        service_3(request=1)

        while (len(self.trace_dic[0]) == 0) and \
                (len(self.trace_dic[1]) == 0) and \
                (len(self.trace_dic[2]) == 0):
            rospy.logwarn('Waiting for the trace callback from all agents')

        if self.task_allocate(style="Global"):
            return "Global", True

        rospy.logerr("LTL Planner: No valid plan has been found for level 2!")
        return "Error", False


    def replan_level_3(self):
        #Update the TS info
        self.update_info["added"] = set()
        self.update_info["relabel"] = set()
        self.update_info["deleted"] = self.team.find_deleted_ts_update(self.trace_dic, self.local_replan_rname, self.plans)

        #Try local replanning first
        if self.task_allocate(style="Local_ts_update"):
            self.plans.state_sequence[self.local_replan_rname] = [(self.local_replan_rname, tt[0], tt[1]) for tt in self.local_plan.prefix]
            return "Local", True

        #TODO: Add ros service for requesting the synchronization
        service_1 = rospy.ServiceProxy('/dr_0/synchronization_service', LTLTrace)
        service_1(request=1)
        service_2 = rospy.ServiceProxy('/a1_gazebo/synchronization_service', LTLTrace)
        service_2(request=1)
        service_3 = rospy.ServiceProxy('/wassi_0/synchronization_service', LTLTrace)
        service_3(request=1)

        while (len(self.trace_dic[0]) == 0) and \
                (len(self.trace_dic[1]) == 0) and \
                (len(self.trace_dic[2]) == 0):
            rospy.logwarn('Waiting for the trace callback from all agents')

        if self.task_allocate(style="Global"):
            return "Global", True

        rospy.logerr("LTL Planner: No valid plan has been found for level 3!")
        return "Error", False