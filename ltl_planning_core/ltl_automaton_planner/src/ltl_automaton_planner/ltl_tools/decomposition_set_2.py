import rospy
import networkx as nx
from networkx.algorithms.simple_paths import is_simple_path, _all_simple_paths_graph, _all_simple_paths_multigraph
from networkx.classes.digraph import DiGraph
from ltl_automaton_planner.ltl_tools.buchi import check_label_for_buchi_edge


def get_decomposition_set(buchi):
    decomp_set = list()
    initial = buchi.graph['initial']
    final = buchi.graph['accept']
    if (not len(initial) == 1):
        rospy.logerr("INITIAL AND FINAL DIMENSION ERROR")
    decomp_set += [initial[0]]
    decomp_set += [final[0]]
    debug1 = buchi.nodes
    for b_node in buchi.nodes:
        if b_node in decomp_set:
            continue

        try:
            # seq_1 = nx.shortest_path(buchi, source=initial[0], target=b_node)
            # seq_2 = nx.shortest_path(buchi, source=b_node, target=final[2])
            # edge_path_1 = zip(seq_1[0:-1], seq_1[1:])
            # edge_path_2 = zip(seq_2[0:-1], seq_2[1:])
            seq_1 = nx.all_simple_paths(buchi, initial[0], b_node)
            seq_2 = nx.all_simple_paths(buchi, b_node, final[0])
            tau_1 = list()
            tau_2 = list()
            # for path_1 in map(nx.utils.pairwise, seq_1):
            path_1 = map(nx.utils.pairwise, seq_1)[0]
            for edge in path_1:
                aa = buchi.edges[edge]
                tau_1 += [buchi.edges[edge]['guard_formula']]

            # for path_2 in map(nx.utils.pairwise, seq_2):
            path_2 = map(nx.utils.pairwise, seq_2)[0]
            for edge in path_2:
                tau_2 += [buchi.edges[edge]['guard_formula']]

            tau = tau_2 + tau_1
            current_node = initial[0]
            essential_run = list()
            essential_run += [current_node]


            for tau_00 in tau:
                selected = False
                potential_node = None
                for succ in buchi.successors(current_node):
                    debug0 = [buchi.edges[current_node, succ]['guard_formula']]
                    transition_can = buchi.edges[current_node, succ]['guard_formula']
                    if transition_can == tau_00:
                        essential_run += [succ]
                        current_node = unicode(succ)
                        selected = True
                        break
                    if transition_can == unicode("(1)") or transition_can == "1":
                        potential_node = unicode(succ)

                if (not selected) and potential_node is not None:
                    essential_run += [potential_node]
                    selected = True
                    potential_node = None

                if not selected:
                    rospy.logerr("ERROR WHEN EXPANDING THE SEQUENCE")
                    print(b_node)
                    print('Current node:')
                    print(current_node)

            # if check_current_pair(buchi, b_node, initial, final[0]):
            #     decomp_set += [b_node]

            # for f in final:
            #     if check_current_pair(buchi, b_node, initial, f):
            #         decomp_set += [b_node]
            #         break

        except:
            rospy.logerr("ERROR WHEN DETERMINING THE DECOMPOSITION SET:")
            print(b_node)
            continue
    return decomp_set


def check_current_pair(buchi, b_node, initial, final):
    seq_1 = nx.shortest_path(buchi, source=initial, target=b_node)
    seq_2 = nx.shortest_path(buchi, source=b_node, target=final)
    edge_path_1 = zip(seq_1[0:-1], seq_1[1:])
    edge_path_2 = zip(seq_2[0:-1], seq_2[1:])
    # seq_1 = nx.all_simple_paths(buchi, initial[0], b_node)
    # seq_2 = nx.all_simple_paths(buchi, b_node, final[0])
    tau_1 = list()
    tau_2 = list()
    # for path_1 in map(nx.utils.pairwise, seq_1):
    # path_1 = map(nx.utils.pairwise, seq_1)[0]
    for edge in edge_path_1:
        aa = buchi.edges[edge]
        tau_1 += [buchi.edges[edge]['guard_formula']]

    # for path_2 in map(nx.utils.pairwise, seq_2):
    path_2 = map(nx.utils.pairwise, seq_2)[0]
    for edge in edge_path_2:
        tau_2 += [buchi.edges[edge]['guard_formula']]

    tau = tau_2 + tau_1
    current_node = initial[0]
    essential_run = list()
    essential_run += [current_node]


    for tau_00 in tau:
        selected = False
        potential_node = None
        for succ in buchi.successors(current_node):
            debug0 = [buchi.edges[current_node, succ]['guard_formula']]
            transition_can = buchi.edges[current_node, succ]['guard_formula']
            if transition_can == tau_00:
                essential_run += [succ]
                current_node = unicode(succ)
                selected = True
                break
            if transition_can == unicode("(1)") or transition_can == "1":
                potential_node = unicode(succ)

        if (not selected) and potential_node is not None:
            essential_run += [potential_node]
            selected = True
            potential_node = None

        if not selected:
            rospy.logerr("ERROR WHEN EXPANDING THE SEQUENCE")
            print(b_node)
            print('Current node:')
            print(current_node)

    if final in essential_run:
        return True

    return False
