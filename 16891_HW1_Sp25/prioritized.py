import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

        self.time_horizon = sum(cell is False for row in self.my_map for cell in row)
        self.counter = 0

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        # constraints = [{'agent': 1, 'loc': [(1,2),(1,3)], 'timestep': 1}]
        constraints = []

    
        for i in range(self.num_of_agents):  # Find path for each agent

            if self.counter >= 500:
                print("exceeded")
                break

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)

            self.counter += 1

            if path is None:
                raise BaseException('No solutions')
            
            if len(path) > (self.time_horizon*2):
                print("no solution exists")
                raise BaseException('No solutions')

            result.append(path)

            ##############################
            # Task 1.3/1.4/2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################

            # timestep at goal for higher priority agent
            goal_t = len(path)-1

            for agent in range(i+1, self.num_of_agents):
                for timestep in range(len(path) - 1):
                    constraints.append({'agent': agent, 'loc': [path[timestep]], 'timestep': timestep})
                    constraints.append({'agent': agent, 'loc': [path[timestep + 1], path[timestep]], 'timestep': timestep + 1})
                for t in range(goal_t, self.time_horizon*2):
                    constraints.append({'agent': agent, 'loc': [self.goals[i]], 'timestep': t})
            

        self.CPU_time = timer.time() - start_time
        

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
