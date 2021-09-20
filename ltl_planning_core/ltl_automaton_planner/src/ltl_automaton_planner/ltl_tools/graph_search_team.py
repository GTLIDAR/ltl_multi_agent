import rospy
from ltl_automaton_planner.ltl_tools.discrete_plan import compute_path_from_pre
from ltl_automaton_planner.ltl_tools.team import Team_Run
from ltl_automaton_planner.ltl_tools.product import ProdAut_Run
import networkx as nx
import time


def compute_team_plans_multi_objective(team):
    # Perform a single-objective search for the multi-robot task allocation
    start = time.time()
    plans = {}
    runs = {}
    team_init = team.graph['initial']
    team_finals = team.graph['accept']

    #Algorithm overview:
    # Since there is only ONE 'initial' node but SEVERAL 'final' nodes, we have to iterate
    # all possible paths leading to every single 'final' nodes. First, generate all possible
    # paths leading to a selected 'final' node. Then, generate a cost for that path. The
    # cost for that path should be the max cost amongst all agents of the same path.
    # If this max cost is lower than the stored highest cost, swap the cost along with
    # the stored path.

    #Create a new variable for storing the 'best' path (sequences of state (int)nums)
    best_path = []
    agent = 0
    curr_cost = 0
    curr_max_agent_cost = 0
    found_better_cost = False
    best_max_agent_cost = float('inf')
    best_cost_list = float('inf')
    curr_cost_list = []
    best_cost_list = []

    #Code below assumes team_init is size 1. This must be confirmed.
    if len(team_init) != 1 :
        rospy.logdebug('team_init set variable not length of 1')

    #Loop over each final node
    for final in team_finals:
        #Generate all possible paths from init to the chosen final node,
        for path in nx.all_simple_paths(team, list(team_init)[0], final):
            #Set prev_node to first node in path
            prev_node = path[0]
            #Retrieve the maximum cost for an agent for each path starting at second node
            for node in path[1:]:
                #If the agent number did not change, extract cost and sum up in agent value
                if team.projection(node)[0] == agent :
                    if team[prev_node][node]['action'] == 'switch_transition':
                        rospy.logdebug('switch transition detected out of sync')
                    curr_cost += team[prev_node][node]['weight']
                else:
                    #Store the highest agent cost
                    if curr_max_agent_cost < curr_cost:
                        curr_max_agent_cost = curr_cost
                        curr_cost_list.append(curr_cost)
                    #Reset
                    curr_cost = 0
                    #Increment
                    agent += 1
                #Set current node to previous
                prev_node = node

            #Set to true if better cost is found
            found_better_cost = False

            #Once all nodes in the path has been analyzed, compare curr_max_agent_cost
            if curr_max_agent_cost < best_max_agent_cost:
                found_better_cost = True
            #If the current best cost is equal to the candidate cost
            elif curr_max_agent_cost == best_max_agent_cost:
                #Compare the total cost first
                total_curr_cost = 0
                for cost in curr_cost_list:
                    total_curr_cost += cost
                #If the current total cost is less, store it
                if total_curr_cost < total_best_cost:
                    found_better_cost = True
                #If total costs are equal, compare the second highest...etc
                elif total_curr_cost == total_best_cost:
                    temp_best_cost_list = list(best_cost_list)
                    temp_curr_cost_list = list(curr_cost_list)
                    temp_best_cost_list.sort()
                    temp_curr_cost_list.sort()
                    for i in range(0,len(temp_best_cost_list)):
                        if temp_best_cost_list[i] > temp_curr_cost_list[i]:
                            found_better_cost = True
                            break
                        #If the best values are less than the candidate, stop
                        elif temp_best_cost_list[i] < temp_curr_cost_list[i]:
                            break
            #Evaluates to true if better cost is found
            if found_better_cost:
                best_path = path
                best_cost_list = list(curr_cost_list)
                best_max_agent_cost = curr_max_agent_cost
                total_best_cost = 0
                for cost in best_cost_list:
                    total_best_cost += cost
            #Reset
            agent = 0

    if best_max_agent_cost != float('inf'):
        print("final path is: ")
        print(best_path)
        print("total best cost is: ")
        for cost in best_cost_list:
            total_best_cost += cost
        print(total_best_cost)
        run = Team_Run(team, best_path, total_best_cost)
        rospy.logdebug('Dijkstra team search done within %.2fs' %(time.time()-start))
        return run, time.time()-start
    else:
        rospy.logerr('No accepting run found in optimal planning!')
        return None, None

def compute_team_plans(team):
    # Perform a single-objective search for the multi-robot task allocation
    start = time.time()
    plans = {}
    runs = {}
    team_init = team.graph['initial']
    team_finals = team.graph['accept']
    rospy.logwarn('Dijkstra team global search start')
    for t_init in team_init:
        plan_pre, plan_dist = nx.dijkstra_predecessor_and_distance(team, t_init)
        for target in team_finals:
            if target in plan_dist:
                plans[target] = plan_dist[target]

        if plans:
            opti_targ = min(plans, key=plans.get)
            plan = compute_path_from_pre(plan_pre, opti_targ)
            plan_cost = plan_dist[opti_targ]
            runs[(t_init, opti_targ)] = (plan, plan_cost)

    if runs:
        plan, plan_cost = min(runs.values(), key=lambda p: p[1])
        run = Team_Run(team, plan, plan_cost)
        rospy.logwarn('Dijkstra team global search done within %.2fs' %(time.time()-start))
        return run, time.time()-start

    rospy.logerr('No accepting run found in optimal planning!')
    return None, None

def compute_local_plan(team, rname):
    start = time.time()
    plans = {}
    runs = {}
    curr_prod = team.graph['pro_list'][rname]
    updated_init = curr_prod.graph['updated_initial']
    updated_accept = curr_prod.graph['updated_accept']
    for t_init in updated_init:
        plan_pre, plan_dist = nx.dijkstra_predecessor_and_distance(curr_prod, t_init)
        for target in updated_accept:
            if target in plan_dist:
                plans[target] = plan_dist[target]

            if plans:
                opti_targ = min(plans, key=plans.get)
                plan = compute_path_from_pre(plan_pre, opti_targ)
                plan_cost = plan_dist[opti_targ]
                runs[(t_init, opti_targ)] = (plan, plan_cost)

    if runs:
        plan, plan_cost = min(runs.values(), key=lambda p: p[1])
        run = ProdAut_Run(curr_prod, plan, plan_cost)
        rospy.logwarn('Dijkstra local PA search done within %.2fs' %(time.time()-start))
        return run, time.time()-start

    # rospy.logerr('No accepting run found in optimal planning!')
    return None, None

def find_reusable_plan(team, rname, old_run):
    start = time.time()
    local_pa_plan = old_run.state_sequence[rname]
    curr_prod = team.graph['pro_list'][rname]
    updated_init = curr_prod.graph['updated_initial']
    new_local_plan = list()
    for i in range(len(local_pa_plan)):
        if updated_init == local_pa_plan[i]:
            new_local_plan = local_pa_plan[i:]
            rospy.logwarn('Reusable path found within %.2fs' %(time.time()-start))
            break

    if len(new_local_plan) != 0:
        run = ProdAut_Run(curr_prod, new_local_plan, 0)  # cost is hardcoded for now since not used
        return run, time.time()-start

    return None, None