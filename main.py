from search import *
import ptvsd

# 5678 is the default attach port in the VS Code debug configurations
""" print("Waiting for debugger attach")
ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
ptvsd.wait_for_attach() """

class MyProblem(Problem):
    """ Searching from a node to another in a given graph """
    def __init__(self, initial, goal, graph):
        Problem.__init__(self, initial, goal)
        self.graph = graph

    def actions(self, A):
        """ The nodes that are reachable from a certain state """
        return list(self.graph.get(A).keys())

    def result(self, state, action):
        """ The result of an action(walking to a neighbour),
        is just that neighbour """
        return action

    def path_cost(self, cost_so_far, A, action, B):
        return cost_so_far + (self.graph.get(A,B) or infinity)

    def find_min_edge(self):
        m = infinity
        for d in self.graph.graph_dict.values():
            local_min = min(d.values())
            m = min(m, local_min)
        return m

    def h(self, node):
        """h function is straight-line distance from a node's state to goal."""
        locs = getattr(self.graph, 'locations', None)
        if locs:
            if type(node) is str:
                return int(distance(locs[node.state], locs[self.goal]))

            return int(distance(locs[node.state], locs[self.goal]))
        else:
            return infinity



def main():

    # Task 1: Romania problem, from Arad to Bucharest
    x = int(input("Task 1 or 2? (1/2): "))

    if x == 1:
        romania_problem = MyProblem('Arad', 'Bucharest', romania_map)

        print("--- Breadth First Search ---")
        print("Nodes visited:")
        bfs = breadth_first_graph_search(romania_problem)
        print("Optimal way (initial_state not shown):")
        print(bfs.solution())
        #print("Path cost: {0}".format(bfs.path_cost()))

        print("\n--- Depth First Search ---")
        print("Nodes visited:")
        dfs = depth_first_graph_search(romania_problem)
        print("Optimal way (initial_state not shown):")
        print(dfs.solution())

        print("\n--- Uniform Cost Search ---")
        print("Nodes visited:")
        ucs = uniform_cost_search(romania_problem)
        print("Optimal way (initial_state not shown):")
        print(ucs.solution())

        print("\n--- Best First Search ---")
        print("Nodes visited:")
        bestfs = best_first_graph_search(romania_problem, lambda node: node.state)
        print("Optimal way (initial_state not shown):")
        print(bestfs.solution())

        print("\n--- A* Search: ---")
        print("Nodes visited:")
        astar = astar_search(romania_problem)
        print("Optimal way (initial_state not shown):")
        print(astar.solution())
    else:
        # Task 2: A farmer, wolf, goat and cabbage problem

        # cost = 1 equals a path that is allowed, if the cost = infinity it is a path to a state that is NOT allowed
        # a = initial_state, q = goal_state
        river_graph = UndirectedGraph(dict(
            a = dict(b = 1000, c = 1000, d = 1, e = 1000),
            d = dict(f = 1),
            f = dict(d = 1, g = 1, h = 1),
            g = dict(i = 1000, j = 1),
            h = dict(k = 1, l = 1000),
            j = dict(m = 1000, n = 1),
            k = dict(n = 1, o = 1000),
            n = dict(p = 1),
            p = dict(q = 1),
            q = dict(p = 1)
        ))
        river_graph.locations = dict(
            a = (7,0), b = (1000,0), c = (1000,0),
            d = (6,0), e = (1000,0), f = (5,0),
            g = (4,0), h = (4,0), i = (1000,0),
            j = (3,0), k = (3,0), l = (1000,0),
            m = (1000, 0), n = (2,0), o = (1000,0),
            p = (1,0), q = (0,0)
        )

        river_problem = MyProblem('a','q', river_graph)

        print("--- Breadth First Search ---")
        print("Nodes visited:")
        bfs = breadth_first_graph_search(river_problem)
        print("Optimal way (initial_state not shown):")
        print(bfs.solution())

        print("\n--- Depth First Search ---")
        print("Nodes visited:")
        dfs = depth_first_graph_search(river_problem)
        print("Optimal way (initial_state not shown):")
        print(dfs.solution())

        """ print("\n--- A* Search ---")
        print("Nodes visited:")
        astar = astar_search(river_problem, lambda node: node.state)
        print("Optimal way (initial_state not shown):")
        print(astar.solution()) """



if __name__ == '__main__':
    main()