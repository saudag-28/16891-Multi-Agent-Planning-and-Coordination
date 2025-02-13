import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def move_joint_state(locs, dir):

    new_locs = []

    for i in range(len(locs)):
        new_locs.append((locs[i][0] + dir[i][0], locs[i][1] + dir[i][1]))

    return new_locs


def generate_motions_recursive(num_agents, cur_agent):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]

    joint_state_motions = []

    if (cur_agent == num_agents):
        return [[]]

    sub_motions = generate_motions_recursive(num_agents, cur_agent+1)
    
    for direction in directions:
        for sub_motion in sub_motions:
            joint_state_motions.append([direction] + sub_motion)

    return joint_state_motions


# 1.1 this is for composite agent
def is_valid_motion(old_loc, new_loc):
    ##############################
    # Task 1.3/1.4: Check if a move from old_loc to new_loc is valid
    # Check if two agents are in the same location (vertex collision)
    # TODO
    for i in range(len(new_loc)):
        for j in range(i+1, len(new_loc)):
            if new_loc[i] == new_loc[j]:
                return False

    # Check edge collision
    # TODO
    for i in range(len(new_loc)):
        for j in range(i+1, len(new_loc)):
            if old_loc[i] == new_loc[j] and old_loc[j] == new_loc[i]:
                return False

    return True


def get_sum_of_cost(paths):
    rst = 0
    if paths is None:
        return -1
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3/1.4: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraint_table = [[]]

    for constraint in constraints:
        if constraint['agent'] == agent:
            time_step = constraint['timestep']
            while len(constraint_table)<=time_step:
                constraint_table.append([])
            constraint_table[time_step].append(constraint)
    
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3/1.4: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # if next_time not in constraint_table:
    #     return False
    
    if next_time < len(constraint_table):
        for constraint in constraint_table[next_time]:
            # vertex constraint - check if this agent is not allowed to be at this vertex at this time step
            # print(f"loc test_1: {constraint['loc']}") # [(1,5)]
            # print(f"loc test_2: {constraint['loc'][0]}") # (1,5)
            # print(f"next loc is: {next_loc}") # (2,3)

            if (len(constraint['loc']) == 1 and constraint['loc'][0] == next_loc) or (len(constraint['loc']) == 2 and constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc):
                # print("Vertex Constraint found: ", constraint)
                return True

    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def in_map(map, loc):
    if loc[0] >= len(map) or loc[1] >= len(map[0]) or min(loc) < 0:
        return False
    else:
        return True


def all_in_map(map, locs):
    for loc in locs:
        if not in_map(map, loc):
            return False
    return True


def future_constrained(goal_loc, curr_time, constraint_table):
    # Case 1: check if this location for this lower priority agent is in the constraint table in the future timestep

    # find the max timestep from the constraint table of this agent
    max_t = len(constraint_table)
    for t in range(curr_time, max_t):
        # for all constraints at this time step
        for constraint in constraint_table[t]:
            if goal_loc == constraint['loc'][0]:
                # print("Continue searching the path")
                return True
    
    return False


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        h_values    - precomputed heuristic values for each location on the map
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.2/1.3/1.4: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    # building the constraint table for this agent
    constraint_table = build_constraint_table(constraints, agent)

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 2.2: Adjust the goal test condition to handle goal constraints
        # TODO 
        if curr['loc'] == goal_loc and not future_constrained(goal_loc, curr['timestep'], constraint_table):

            path_ = get_path(curr)
            return path_

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if not in_map(my_map, child_loc) or my_map[child_loc[0]][child_loc[1]]:
                continue

            # check for constraints before creating the child node
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                continue

            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
        
    return None  # Failed to find solutions


def joint_state_a_star(my_map, starts, goals, h_values, num_agents):
    """ my_map      - binary obstacle map
        start_loc   - start positions
        goal_loc    - goal positions
        num_agent   - total number of agents in fleet
    """

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = 0
    ##############################
    # Task 1.1: Iterate through starts and use list of h_values to calculate total h_value for root node
    #
    # TODO

    h_value = sum(h_values[i].get(starts[i], 0) for i in range(len(starts)))
        

    root = {'loc': starts, 'g_val': 0, 'h_val': h_value, 'parent': None}
    push_node(open_list, root)
    closed_list[tuple(root['loc'])] = root

    ##############################
    # Task 1.1:  Generate set of all possible motions in joint state space
    #
    #
    # TODO
    directions = generate_motions_recursive(num_agents, 0)

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['loc'] == goals:
            return get_path(curr)

        for dir in directions:

            ##############################
            # Task 1.1:  Update position of each agent
            #
            # TODO
            child_loc = move_joint_state(curr['loc'], dir)

            if not all_in_map(my_map, child_loc):
                continue
            ##############################
            # Task 1.1:  Check if any agent is in an obstacle
            #
            valid_move = True
            # TODO
            for loc in child_loc:
                if not in_map(my_map, loc) or my_map[loc[0]][loc[1]]:
                    valid_move = False
                    break 

            if not valid_move:
                continue

            ##############################
            # Task 1.1:   check for collisions
            #
            # TODO
            if not is_valid_motion(curr['loc'], child_loc):
                continue

            ##############################
            # Task 1.1:  Calculate heuristic value
            #
            # TODO
            h_value = 0
            for i in range(num_agents):
                h_value = sum(h_values[i].get(child_loc[i], 0) for i in range(num_agents))

            # Create child node
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + num_agents,
                     'h_val': h_value,
                     'parent': curr}
            if tuple(child['loc']) in closed_list:
                existing_node = closed_list[tuple(child['loc'])]
                if compare_nodes(child, existing_node):
                    closed_list[tuple(child['loc'])] = child
                    push_node(open_list, child)
            else:
                closed_list[tuple(child['loc'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
