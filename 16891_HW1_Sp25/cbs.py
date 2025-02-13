import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import copy

def detect_first_collision_for_path_pair(path1, path2):
    ##############################
    # Task 2.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    # find the path with minimum length
    timesteps =  max(len(path1), len(path2))
    for t in range(timesteps-1):
        p1_loc = get_location(path1, t)
        p2_loc = get_location(path2, t)
        p1_loc_next = get_location(path1, t+1)
        p2_loc_next = get_location(path2, t+1)

        # print(f"p1_loc: {p1_loc}")
        # print(f"p2_loc: {p2_loc}")
        # print(f"p1_loc_next: {p1_loc_next}")
        # print(f"p2_loc_next: {p2_loc_next}")
        # print(" ")

        # vertex collision
        if p1_loc == p2_loc:
            # print(f"return collision: {{'loc': [p1_loc], 'timestep': t}}")
            return {'loc': [p1_loc], 'timestep': t}

        # edge collision
        if p1_loc == p2_loc_next and p1_loc_next == p2_loc:
            # print(f"returning collision: {{'loc': [p1_loc, p2_loc], 'timestep': t+1}}")
            return {'loc': [p1_loc, p2_loc], 'timestep': t+1}
    
    # print("No collisions found for these two agents")
    return None
            

def detect_collisions_among_all_paths(paths):
    ##############################
    # Task 2.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []

    # print(f"len of paths: {len(paths)}")

    for i in range(len(paths)): # if length is 2 then it will loop from - 0, 1
        for j in range(i+1, len(paths)): # it starts from i+1 till 1
            coll_agents = {'a1': i, 'a2': j}
            # print(f"finding collisions for agent {i} and {j}")
            collision = detect_first_collision_for_path_pair(paths[i], paths[j])
            if collision:
                coll_agents.update(collision)
                collisions.append(coll_agents)

                # print(f"collisions: {coll_agents}")
    
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 2.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constraints = []

    # vertex collision
    if len(collision['loc']) == 1:
        constraints.append({
            'agent': collision['a1'],
            'loc': [collision['loc'][0]],
            'timestep': collision['timestep']
        })

        constraints.append({
            'agent': collision['a2'],
            'loc': [collision['loc'][0]],
            'timestep': collision['timestep']
        })

    # edge collision
    if len(collision['loc']) == 2:

        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]

        constraints.append({
            'agent': collision['a1'],
            'loc': [loc1, loc2],
            'timestep': collision['timestep']
        })

        constraints.append({
            'agent': collision['a2'],
            'loc': [loc2, loc1],
            'timestep': collision['timestep']
        })

    return constraints

    
class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations

        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])

            print(f"path for agent {i}: {path}")

            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths']) # line 4
        root['collisions'] = detect_collisions_among_all_paths(root['paths']) # line 3
        self.push_node(root) # line 5

        # Task 2.1: Testing
        print(root['collisions'])

        # Task 2.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))


        ##############################
        # Task 2.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # These are just to print debug output - can be modified once you implement the high-level search

        while self.open_list:
            node_p = self.pop_node()

            if len(node_p['collisions']) == 0:
                print("no collisions found, therefore goal node")
                # self.print_results(node_p)
                return node_p['paths']
            
            # pick one collision from collisions
            collision_ = node_p['collisions'][0]
            
            # convert it into constraints
            constraints_ = standard_splitting(collision_)

            # Add a new child node to your open list for each constraint (total = 2)
            for constraint in constraints_:
                
                # create a copy of node p
                node_q = copy.deepcopy(node_p)
                # append the new constraints to the new node
                node_q['constraints'].append(constraint)

                # the agent in constraint
                a_i = constraint['agent']
                # print(f"agent: {a_i}")
                # replan the path
                path_ = a_star(self.my_map, self.starts[a_i], self.goals[a_i], self.heuristics[a_i],
                          a_i, node_q['constraints'])
                
                # print(f"path_: {path_}")
                

                if path_ is None:
                    continue

                # replace the path od agent a_i in node_q's plan by path
                node_q['paths'][a_i] = path_
                node_q['cost'] = get_sum_of_cost(node_q['paths'])
                node_q['collisions'] = detect_collisions_among_all_paths(node_q['paths'])

                # print(f"new node_q: {node_q}")

                self.push_node(node_q)
            
            # exit()

    
        # self.print_results(root)
        return root['paths']
        

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
