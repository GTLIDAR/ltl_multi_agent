import rospy

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.product import ProdAut
from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner

from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg
from ltl_automaton_planner.ltl_tools.buchi import mission_to_buchi
from ltl_automaton_planner.ltl_tools.decomposition_set import get_decomposition_set
import networkx as nx
import matplotlib.pyplot as plt

def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    plt.show()
    return

class MainTest(object):
    def __init__(self):
        self.init_params()

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
        transition_system_textfile = rospy.get_param('transition_system_mobile_textfile')
        self.initial_state_ts_dict = None
        self.transition_system = import_ts_from_file(transition_system_textfile)
        state_models = state_models_from_ts(self.transition_system, self.initial_state_ts_dict)

        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = TSModel(state_models)

        # new wrapper starts from here
        self.buchi = mission_to_buchi(self.hard_task, self.soft_task)
        self.decompose_set = get_decomposition_set(self.buchi)

        self.product_1 = ProdAut(self.robot_model, self.buchi, 1000)
        self.product_1.graph['ts'].build_full()
        self.product_1.build_full()

        self.product_2 = ProdAut(self.robot_model, self.buchi, 1000)
        self.product_2.graph['ts'].build_full()
        self.product_2.build_full()

        self.pro_list = [self.product_1, self.product_2]
        self.team_size = len(self.pro_list)

        # append the robot number into the current nodes
        # for node in buchi_1.nodes:

        self.T = nx.DiGraph(initials=set(), initial=set(), accept=set())
        for idx, prod in enumerate(self.pro_list):
            for node in prod.nodes:
                ts_node = prod.nodes[node]['ts']
                buchi_node = prod.nodes[node]['buchi']
                team_node = (idx, ts_node, buchi_node)
                if not self.T.has_node(team_node):
                    self.T.add_node(team_node, rname=idx, ts=ts_node, buchi=buchi_node, label=ts_node)

                    if (prod.nodes[node]['ts'] in prod.graph['ts'].graph['initial']) and (prod.nodes[node]['buchi'] in prod.graph['buchi'].graph['initial']):
                        self.T.graph['initials'].add(team_node)
                        if idx == 0:
                            self.T.graph['initial'].add(team_node)

                    if prod.nodes[node]['buchi'] in prod.graph['buchi'].graph['accept']:
                        self.T.graph['accept'].add(team_node)

                for suc in prod.successors(node):
                    ts_node_suc = prod.nodes[suc]['ts']
                    buchi_node_suc = prod.nodes[suc]['buchi']
                    team_node_suc = (idx, ts_node_suc, buchi_node_suc)
                    if not self.T.has_node(team_node_suc):
                        self.T.add_node(team_node_suc, rname=idx, ts=ts_node_suc, buchi=buchi_node_suc, label=ts_node_suc)

                        if (prod.nodes[suc]['ts'] in prod.graph['ts'].graph['initial']) and (prod.nodes[suc]['buchi'] in prod.graph['buchi'].graph['initial']):
                            self.T.graph['initials'].add(team_node_suc)
                            if idx == 0:
                                self.T.graph['initial'].add(team_node_suc)

                        if prod.nodes[suc]['buchi'] in prod.graph['buchi'].graph['accept']:
                            self.T.graph['accept'].add(team_node_suc)

                    cost = prod.graph['ts'][ts_node][ts_node_suc]['weight']
                    action = prod.graph['ts'][ts_node][ts_node_suc]['action']
                    self.T.add_edge(team_node, team_node_suc, transition_cost=cost, action=action)

        # Add switch transition between different robots
        for node in self.T.nodes:
            if self.T.nodes[node]['rname'] == self.team_size-1:
                continue

            if self.T.nodes[node]['buchi'] in self.decompose_set:
                next_rname = self.T.nodes[node]['rname'] + 1
                next_ts_state = None
                for next_team_init in self.T.graph['initials']:
                    if next_team_init[0] == next_rname:
                        next_ts_state = next_team_init[1]

                if next_ts_state == None:
                    raise AssertionError()

                next_buchi_state = self.T.nodes[node]['buchi']
                next_node = (next_rname, next_ts_state, next_buchi_state)

                # Check self-transition and the respective condition is fulfilled
                label = self.pro_list[self.T.nodes[node]['rname']].graph['ts'].nodes[self.T.nodes[node]['ts']]['label']
                guard = self.pro_list[self.T.nodes[node]['rname']].graph['buchi'].edges[next_buchi_state, next_buchi_state]['guard']

                if guard.check(label):
                    self.T.add_edge(node, next_node, transition_cost=0, action='switch_transition')



        # show_automaton(self.product_1)
        # self.team = nx.union(self.product_1.graph['ts'], self.product_2.graph['buchi'])

        # plt.subplot(131)
        # show_automaton(buchi_1)
        # plt.subplot(132)
        # show_automaton(buchi_2)
        # plt.subplot(133)
        show_automaton(self.T)
        plt.show()



#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=False)
    try:
        ltl_planner_node = MainTest()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Planner: "+str(e))
        rospy.logerr("LTL Planner: shutting down...")