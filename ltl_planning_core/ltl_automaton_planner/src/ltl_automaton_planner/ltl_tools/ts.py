# -*- coding: utf-8 -*-
import rospy
from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard

from math import sqrt
from itertools import product
from networkx.classes.digraph import DiGraph

import networkx as nx

###########################################################
# Construct a single TSModel;
# Input is a combination of one/multiple transition systems
###########################################################

class TSModel(DiGraph):

    def __init__(self, state_models):
        """
        TS model, built from a list of state models to combine.
        """

        self.state_models = state_models

    def build_full(self):
        """
        Build TS graph from one or more state model TS.
        """

        # If only one state model, use directly as the TS
        if len(self.state_models) == 1:
            DiGraph.__init__(self, 
                             incoming_graph_data=self.state_models[0],
                             initial=self.state_models[0].graph['initial'],
                             ts_state_format=self.state_models[0].graph['ts_state_format'])

        # If more than one, build a combined TS model
        else:
            # Build digraph object
            DiGraph.__init__(self,
                             initial=set(),
                             ts_state_format=[model.graph['ts_state_format'] for model in self.state_models])
            # Compose and add nodes
            self.compose_nodes(self.state_models)
            # Compose and add edges between nodes
            self.compose_edges(self.state_models)
            # Compose initial state
            self.compose_initial(self.state_models)

        rospy.loginfo("LTL Planner: full model constructed with %d states and %s transitions" %(len(self.nodes()), len(self.edges())))
        rospy.loginfo("LTL Planner: initial state in TS is %s" %str(self.graph['initial']))


    def set_initial(self, ts_state):
        """
        Delete and set new initial state.
        """

        # If state exist in graph, change initial and return true
        if ts_state in self.nodes():
            self.graph['initial'] = set([ts_state])
            return True
        # If state doesn't exist in graph, return false
        else:
            return False

    def compose_initial(self, graph_list):
        """
        Compose and set initial state

        Create products of initial nodes from the input graph list.

        """
        initial_states = [list(graph.graph['initial']) for graph in graph_list]
        init_nodes = self.node_product(*initial_states)

        self.graph['initial'].update(set(init_nodes))

    def compose_nodes(self, graph_list):
        """
        Compose and add nodes to the digraph

        Nodes are products of nodes from the input graph list.

        """
        node_product = self.node_product(*graph_list)
        for node in node_product:
            self.add_node(node, label=node, marker='unvisited')

    def compose_edges(self, graph_list):
        """
        Compose and add edges to the digraph

        Nodes are products of nodes from the input graph list. Needs to be called after composing nodes.

        """
        # For each individual state model
        for i in range(len(graph_list)):
            # For each state in this model
            for state in graph_list[i]:
                # Look for node in the product which include this state
                nodes = [elem for elem in self.nodes if elem[i] == state[0]]
                for node in nodes:
                    successor_state_node = list(node)
                    for successor_state in graph_list[i].successors(state):
                        # Create successor node by replacing one state by its successor
                        successor_state_node[i] = successor_state[0]
                        successor_node = tuple(successor_state_node)
                        # Add edge using weight and action label from the state model
                        if self.is_action_allowed(graph_list[i][state][successor_state]['guard'], self.nodes[node]['label']):
                            self.add_edge(node, successor_node,
                                          action=graph_list[i][state][successor_state]['action'],
                                          guard=graph_list[i][state][successor_state]['guard'],
                                          weight=graph_list[i][state][successor_state]['weight'],
                                          marker='visited')

    def is_action_allowed(self, action_guard, ts_label):
        """
        Check action guard against the node label.
        """

        guard_expr = parse_guard(action_guard)
        if guard_expr.check(ts_label):
            return True
        else:
            return False

    @staticmethod
    def node_product(*args):
        """
        Returns a list of product nodes.
            
        Take as input lists of nodes.

        """
        node_pools = [list(pool) for pool in args]
        product_pool = [tuple()]
        for node_pool in node_pools:
            product_pool = [x+y for x in product_pool for y in node_pool]

        return product_pool