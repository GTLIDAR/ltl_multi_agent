import rospy

import networkx as nx
from networkx.classes.digraph import DiGraph
from ltl_automaton_planner.ltl_tools.buchi import check_label_for_buchi_edge

class TeamModel(DiGraph):
    def __init__(self, product_list, decomposition_set):
        DiGraph.__init__(self, pro_list=product_list, decomposition_set=decomposition_set, ts_initials=set(), initial=set(), accept=set())

    def build_team(self):
        # append the robot number into the current nodes
        # for node in buchi_1.nodes:
        for idx, prod in enumerate(self.graph['pro_list']):
            for node in prod.nodes:
                ts_node = prod.nodes[node]['ts']
                buchi_node = prod.nodes[node]['buchi']
                team_node = (idx, ts_node, buchi_node)
                # buchi doesn't have to be initial for all the agents
                ts_initial = (idx, ts_node)
                if not self.has_node(team_node):
                    self.add_node(team_node, rname=idx, ts=ts_node, buchi=buchi_node, label=ts_node)

                    if ts_node in prod.graph['ts'].graph['initial']:
                        self.graph['ts_initials'].add(ts_initial)
                        if idx == 0 and (buchi_node in prod.graph['buchi'].graph['initial']):
                            self.graph['initial'].add(team_node)

                    if prod.nodes[node]['buchi'] in prod.graph['buchi'].graph['accept']:
                        self.graph['accept'].add(team_node)

                for suc in prod.successors(node):
                    ts_node_suc = prod.nodes[suc]['ts']
                    buchi_node_suc = prod.nodes[suc]['buchi']
                    team_node_suc = (idx, ts_node_suc, buchi_node_suc)
                    # buchi doesn't have to be initial for all the agents
                    ts_initial_suc = (idx, ts_node_suc)
                    if not self.has_node(team_node_suc):
                        self.add_node(team_node_suc, rname=idx, ts=ts_node_suc, buchi=buchi_node_suc, label=ts_node_suc)

                        if ts_node_suc in prod.graph['ts'].graph['initial']:
                            self.graph['ts_initials'].add(ts_initial_suc)
                            if idx == 0 and (prod.nodes[suc]['buchi'] in prod.graph['buchi'].graph['initial']):
                                self.graph['initial'].add(team_node_suc)

                        if prod.nodes[suc]['buchi'] in prod.graph['buchi'].graph['accept']:
                            self.graph['accept'].add(team_node_suc)

                    cost = prod.graph['ts'][ts_node][ts_node_suc]['weight']
                    action = prod.graph['ts'][ts_node][ts_node_suc]['action']
                    self.add_edge(team_node, team_node_suc, transition_cost=cost, action=action, weight=cost)

        self.add_switch_transition()

        rospy.loginfo('Decomposition finished: buchi automation contains %d decomposable states' %(len(self.graph['decomposition_set'])))
        rospy.loginfo('LTL Planner Multi Robot: full team model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())))


    def add_switch_transition(self):
        # Add switch transition between different robots
        for node in self.nodes:
            if self.nodes[node]['rname'] == len(self.graph['pro_list'])-1:
                continue

            if self.nodes[node]['buchi'] in self.graph['decomposition_set']:
                next_rname = self.nodes[node]['rname'] + 1
                next_ts_state = None
                for next_team_init in self.graph['ts_initials']:
                    if next_team_init[0] == next_rname:
                        next_ts_state = next_team_init[1]

                if next_ts_state == None:
                    raise AssertionError()

                next_buchi_state = self.nodes[node]['buchi']
                next_node = (next_rname, next_ts_state, next_buchi_state)

                # Check self-transition and the respective condition is fulfilled
                # label = self.graph['pro_list'][self.nodes[node]['rname']].graph['ts'].nodes[self.nodes[node]['ts']]['label']
                #
                # try:
                #     guard = self.graph['pro_list'][self.nodes[node]['rname']].graph['buchi'].edges[next_buchi_state, next_buchi_state]['guard']
                #     if guard.check(label):
                #         self.add_edge(node, next_node, transition_cost=0, action='switch_transition', weight=0)
                # except:
                #     rospy.logwarn('No self-transition at: ')
                #     print(next_buchi_state)

                self.add_edge(node, next_node, transition_cost=0, action='switch_transition', weight=0)


    def remove_switch_transition(self):
        # Remove switch transition between different robots
        remove_list = list()
        for node in self.nodes:
            if self.nodes[node]['rname'] == len(self.graph['pro_list'])-1:
                continue

            if self.nodes[node]['buchi'] in self.graph['decomposition_set']:
                next_rname = self.nodes[node]['rname'] + 1
                next_ts_state = None
                for next_team_init in self.graph['ts_initials']:
                    if next_team_init[0] == next_rname:
                        next_ts_state = next_team_init[1]

                if next_ts_state == None:
                    raise AssertionError()

                next_buchi_state = self.nodes[node]['buchi']
                next_node = (next_rname, next_ts_state, next_buchi_state)

                remove_tuple = (node, next_node)
                remove_list.append(remove_tuple)

        self.remove_edges_from(remove_list)


    def build_team_initial(self):
        #After synchronization the initial states need to be updated
        self.graph['ts_initials'] = set()
        self.graph['initial'] = set()


    def revise_team(self, trace_dic, rname, old_run):
        assert len(trace_dic) == len(self.graph['pro_list'])
        for id in range(len(trace_dic)):
            # build new initials for team model
            new_ts_init = trace_dic[id][-1]
            try:
                self.graph['pro_list'][id].graph['ts'].set_initial(new_ts_init)
            except:
                rospy.logerr('Failed to set initial ts')

        # remove all nodes and edges but keep the graph attributes
        all_nodes = list(self.nodes)
        self.remove_nodes_from(all_nodes)
        all_edges = list(self.edges)
        self.remove_edges_from(all_edges)

        # rebuild the team model given new initials and updated PAs
        self.graph['ts_initials'] = set()
        self.graph['initial'] = set()
        self.graph['accept'] = set()
        self.build_team()

        # add synchronized transitions to record the task status
        for name in range(len(old_run.state_sequence)):
            ts_trace = trace_dic[name]
            pa_plan = old_run.state_sequence[name]
            if len(ts_trace) <= len(pa_plan):
                for ts, pa in zip(ts_trace[:-1], pa_plan):
                    name_, ts_, buchi_ = self.projection(pa)
                    assert ts == ts_
                    assert name == name_

                paused_ts = ts_trace[-1]
                name0, plan_ts, curr_buchi = self.projection(pa_plan[len(ts_trace)-1])
                if name != rname:
                    assert name == name0
                    assert plan_ts == paused_ts
            else:
                name0, plan_ts, curr_buchi = self.projection(pa_plan[-1])

            name00, initi_ts, init_buchi = self.projection(pa_plan[0])
            for n_ in range(len(trace_dic)):
                for ts_node in self.graph['pro_list'][n_].graph['ts'].nodes:
                    init_team_node = (n_, ts_node, init_buchi)
                    init_local_pa_node = (ts_node, init_buchi)
                    curr_team_node = (n_, ts_node, curr_buchi)
                    curr_local_pa_node = (ts_node, curr_buchi)

                    #update team
                    self.add_edge(init_team_node, curr_team_node, transition_cost=0, action='synchronized_transition', weight=0)

                    #update the local PA as well
                    self.graph['pro_list'][n_].add_edge(init_local_pa_node, curr_local_pa_node, transition_cost=0, action='synchronized_transition', weight=0)


        # for name, list in trace_dic.item():
        #     init_node = (name, )
        #
        # #For testing
        # if len(trace_dic) == len(self.graph['pro_list']):
        #     init_node = (0, ('r3',), unicode('T0_init'))
        #     curr_node = (0, ('r3',), unicode('T3_S5'))
        #     self.add_edge(init_node, curr_node, transition_cost=0, action='test_transition', weight=0)
        #
        #     init_node = (0, ('r2',), unicode('T0_init'))
        #     curr_node = (0, ('r2',), unicode('T3_S5'))
        #     self.add_edge(init_node, curr_node, transition_cost=0, action='test_transition', weight=0)
        #
        #     init_node = (0, ('r1',), unicode('T0_init'))
        #     curr_node = (0, ('r1',), unicode('T3_S5'))
        #     self.add_edge(init_node, curr_node, transition_cost=0, action='test_transition', weight=0)
        #
        #     init_node = (1, ('r3',), unicode('T0_init'))
        #     curr_node = (1, ('r3',), unicode('T3_S5'))
        #     self.add_edge(init_node, curr_node, transition_cost=0, action='test_transition', weight=0)
        #
        #     init_node = (1, ('r2',), unicode('T0_init'))
        #     curr_node = (1, ('r2',), unicode('T3_S5'))
        #     self.add_edge(init_node, curr_node, transition_cost=0, action='test_transition', weight=0)
        #
        #     init_node = (1, ('r1',), unicode('T0_init'))
        #     curr_node = (1, ('r1',), unicode('T3_S5'))
        #     self.add_edge(init_node, curr_node, transition_cost=0, action='test_transition', weight=0)
        # else:
        #     rospy.logerr('Number of trace does not match the number of robots')


    def revise_local_pa(self, trace_dic, rname, old_run, update_info):
        self.update_local_pa(trace_dic, rname, old_run, checkLast=True)
        local_pa = self.graph['pro_list'][rname]
        added_pairs = update_info["added"]
        deleted_pairs = update_info["deleted"]
        relabel_states = update_info["relabel"]
        # add transition
        for added_pair in added_pairs:
            for pa_node in local_pa.nodes:
                ts_node, bu_node = local_pa.projection(pa_node)
                if ts_node == added_pair[0]:
                    label = local_pa.graph['ts'].nodes[ts_node]['label']
                    for bu_succ in local_pa.graph['buchi'].successors(bu_node):
                        guard = local_pa.graph['buchi'].edges[bu_node, bu_succ]['guard']
                        if guard.check(label):
                            self.add_edge((ts_node, bu_node), (added_pair[1], bu_succ), transition_cost=0, action='added_transition', weight=0)
                            local_pa.graph['ts'][ts_node][added_pair[1]]['weight'] = 0   #TBD
                            local_pa.graph['ts'][ts_node][added_pair[1]]['action'] = 'added_transition'

        # remove transition
        remove_list = list()
        for deleted_pair in deleted_pairs:
            for pa_node in local_pa.nodes:
                ts_node, bu_node = local_pa.projection(pa_node)
                if ts_node == deleted_pair[0]:
                    label = local_pa.graph['ts'].nodes[ts_node]['label']
                    for bu_succ in local_pa.graph['buchi'].successors(bu_node):
                        guard = local_pa.graph['buchi'].edges[bu_node, bu_succ]['guard']
                        if guard.check(label):
                            remove_list.append(((ts_node, bu_node), (deleted_pair[1], bu_succ)))
        local_pa.remove_edges_from(remove_list)

        # relabel
        remove_list_relabel = list()
        for relabel_state in relabel_states:
            for ts_pre in local_pa.graph['ts'].predecessors(relabel_state[1]):
                for pa_node in local_pa.nodes:
                    ts_node, bu_node = local_pa.projection(pa_node)
                    if ts_node == ts_pre:
                        for bu_succ in local_pa.graph['buchi'].successors(bu_node):
                            guard = local_pa.graph['buchi'].edges[bu_node, bu_succ]['guard']
                            if guard.check(relabel_state[0]):
                                self.add_edge((ts_node, bu_node), (relabel_state[1], bu_succ), transition_cost=0, action='added_transition', weight=0)
                                local_pa.graph['ts'][ts_node][relabel_state[1]]['weight'] = 0   #TBD
                                local_pa.graph['ts'][ts_node][relabel_state[1]]['action'] = 'relabeled_transition'

                for pa_node in local_pa.nodes:
                    ts_node, bu_node = local_pa.projection(pa_node)
                    if ts_node == ts_pre:
                        for bu_succ in local_pa.graph['buchi'].successors(bu_node):
                            guard = local_pa.graph['buchi'].edges[bu_node, bu_succ]['guard']
                            if not guard.check(relabel_state[0]):
                                remove_list_relabel.append(((ts_node, bu_node), (relabel_state[1], bu_succ)))
        local_pa.remove_edges_from(remove_list_relabel)


    def update_local_pa(self, trace_dic, rname, old_run, checkLast=False):
        local_pa_plan = old_run.state_sequence[rname]
        local_ts_trace = trace_dic[rname]
        #zip will stop after one list runs out
        #double check the trace is satisfying the plan

        if checkLast:
            for pa_plan, ts_trace in zip(local_pa_plan, local_ts_trace):
                name, ts, buchi = self.projection(pa_plan)
                assert name == rname
                assert ts == ts_trace

            new_pa_init_set = set()
            name, current_ts, current_buchi = self.projection(local_pa_plan[len(local_ts_trace)-1])
            new_pa_init_set.add((current_ts, current_buchi))

            #Local PA goal is the same
            name, final_ts, final_buchi = self.projection(local_pa_plan[-1])
            self.graph['pro_list'][rname].build_updated_initial_accept(new_pa_init_set, final_buchi)

        else:
            for pa_plan, ts_trace in zip(local_pa_plan, local_ts_trace[:-1]):
                name, ts, buchi = self.projection(pa_plan)
                assert name == rname
                assert ts == ts_trace

            new_pa_init_set = set()
            changed_ts = local_ts_trace[-1]
            if len(local_ts_trace) == 1:
                name, current_ts, current_buchi = self.projection(local_pa_plan[len(local_ts_trace)-1])
                new_pa_init_set.add((changed_ts, current_buchi))
            else:
                name, plan_ts, previous_buchi = self.projection(local_pa_plan[len(local_ts_trace)-2])
                for succ_buchi in self.graph['pro_list'][rname].graph['buchi'].successors(previous_buchi):
                    label = self.graph['pro_list'][rname].graph['ts'].nodes[plan_ts]['label']
                    truth, dist = check_label_for_buchi_edge(self.graph['pro_list'][rname].graph['buchi'], label, previous_buchi, succ_buchi)
                    if truth:
                        new_pa_init_set.add((changed_ts, succ_buchi))

            #Local PA goal is the same
            name, final_ts, final_buchi = self.projection(local_pa_plan[-1])
            self.graph['pro_list'][rname].build_updated_initial_accept(new_pa_init_set, final_buchi)



    def find_deleted_malfunction(self, trace_dic, rname):
        failed_ts_state = trace_dic[rname][-1]
        deleted_set = set()
        for suc_ts in self.graph['pro_list'][rname].graph['ts'].successors(failed_ts_state):
            deleted_set.add((failed_ts_state, suc_ts))
        return deleted_set


    def find_deleted_ts_update(self, trace_dic, rname, old_plan):
        deleted_set = set()
        local_ts_trace = trace_dic[rname]
        name, ts_next, buchi_next = self.projection(old_plan.state_sequence[rname][len(local_ts_trace)])
        ts_curr = trace_dic[rname][-1]
        deleted_set.add((ts_curr, ts_next))
        return deleted_set


    def projection(self, team_node):
        rname = self.nodes[team_node]['rname']
        ts_node = self.nodes[team_node]['ts']
        buchi_node = self.nodes[team_node]['buchi']
        return rname, ts_node, buchi_node


class Team_Run(object):
    def __init__(self, team, plan, plan_cost):
        self.team_plan = plan
        self.totalcost = plan_cost
        self.plan_output(team)

    def plan_output(self, team):
        self.action_sequence = {}
        self.state_sequence = {}
        self.ts_state_sequence = {}
        rname_init = 0
        self.plan_local = list()
        self.ts_plan_local = list()
        for node in self.team_plan:
            # This is allowable since the order of robot is fixed according to switch transition
            rname, ts_node, buchi_node = team.projection(node)
            if rname > rname_init:
                self.state_sequence[rname-1] = self.plan_local
                self.ts_state_sequence[rname-1] = self.ts_plan_local
                self.plan_local = list()
                self.ts_plan_local = list()
                rname_init = rname
            self.plan_local.append(node)
            self.ts_plan_local.append(ts_node)
        self.state_sequence[rname] = self.plan_local
        self.ts_state_sequence[rname] = self.ts_plan_local

        for r_idx, state_seq in self.state_sequence.items():
            team_edges = zip(state_seq[0:-1], state_seq[1:])
            action_sequence_local = list()
            for team_edge in team_edges:
                action_sequence_local.append(team[team_edge[0]][team_edge[1]]['action'])
            self.action_sequence[r_idx] = action_sequence_local

        if len(self.action_sequence) is not len(team.graph['pro_list']):
            rospy.loginfo('LTL Planner: Only the following robots are assigned tasks')

        for r_idx, act_seq in self.action_sequence.items():
            if len(act_seq) == 0:
                rospy.loginfo('LTL Planner: Robot-%d plan: Empty; no task assigned' %r_idx)
            else:
                rospy.loginfo('LTL Planner: Robot-%d plan: ' %r_idx + str(act_seq))

        # if self.ts_plan_sorted:
        #     self.ts_edges_sorted = list()
        #     for idx, ts_nodes in enumerate(self.ts_plan_sorted):
        #         self.ts_edges = zip(ts_nodes[0:-1], ts_nodes[1:])
        #         self.ts_edges_sorted.append(self.ts_edges)
