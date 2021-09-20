import rospy
import networkx as nx
import re
from networkx.algorithms.simple_paths import is_simple_path, _all_simple_paths_graph, _all_simple_paths_multigraph
from networkx.classes.digraph import DiGraph
from ltl_automaton_planner.ltl_tools.buchi import check_label_for_buchi_edge
from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard


def get_decomposition_set(buchi):
    init_nodes = buchi.graph['initial']
    accept_nodes = buchi.graph['accept']
    label_require = {}
    for start, end, label in list(buchi.edges.data("guard_formula")):
        # return dict, key for each sub_clause, and value = (enable_set, disable_set)
        label_require[label] = extract_transition(label)
    decomp_set = set(init_nodes + accept_nodes)

    for b_node in list(buchi.nodes):
        if b_node in decomp_set:
            continue
        for init in init_nodes:
            find = False
            try:
                path_1 = nx.dijkstra_path(buchi, init, b_node)
            except nx.NetworkXNoPath:
                continue
            trace1 = []
            for i in range(len(path_1)-1):
                label = buchi[path_1[i]][path_1[i+1]]["guard_formula"]
                key_list = list(label_require[label].keys())
                trace1.append(list(label_require[label][key_list[0]][0]))

            for accept in accept_nodes:
                try:
                    path2 = nx.dijkstra_path(buchi, b_node, accept)
                except nx.NetworkXNoPath:
                    continue
                # each node related to a list of aps
                trace2 = []
                for i in range(len(path2) - 1):
                    label = buchi[path2[i]][path2[i+1]]["guard_formula"]
                    key_list = list(label_require[label].keys())
                    trace2.append(list(label_require[label][key_list[0]][0]))
                # complete trace

                if trace_accept(trace2 + trace1, buchi, init_nodes[:], accept_nodes[:]):
                    # print(node, trace1, trace2)
                    decomp_set.add(b_node)
                    find = True
                    break
            if find:
                break
    return decomp_set


def trace_accept(trace, ba, init_nodes, accept_nodes):
    queue = init_nodes
    step = 0
    while queue:
        if step < len(trace):
            cur_ap = trace[step]
            step += 1
        else:
            break
        new_queue = []
        visited = set()
        while queue:
            cur = queue.pop()
            if cur in visited:
                continue
            visited.add(cur)
            for node in list(ba[cur]):
                # print(cur, node)
                label = ba[cur][node]["guard_formula"]
                if can_transit(label, cur_ap):
                    # print(can_transit(label, cur_ap), label, cur_ap, cur, node)
                    new_queue.append(node)
        queue = new_queue

    if step < len(trace):
        return False
    accept_set = set(accept_nodes)
    for node in queue:
        if node in accept_set:
            return True
    return False


def extract_transition(label):
    pat_ap = re.compile(r"\w+")
    require = {}
    and_formula = label.split(" || ")
    for f in and_formula:
        ap_formula = f.split(" && ")
        enable, disable = set(), set()
        for ap in ap_formula:
            atom = pat_ap.search(ap).group()
            if "!" in ap:
                disable.add(atom)
            else:
                if atom != "1":
                    enable.add(atom)
        require[f] = (enable, disable)
    return require


def can_transit(label, aps):
    # can speed up by preprocessing. require: dic of dic
    require = extract_transition(label)
    aps_set = set(aps)
    for key in require.keys():
        enable, disable = require[key]
        if aps_set & disable:
            continue
        if enable - aps_set:
            continue
        return True
    return False