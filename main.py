from search import *

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
                return int(distance(locs[node], locs[self.goal]))

            return int(distance(locs[node.state], locs[self.goal]))
        else:
            return infinity



def main():

    romania_problem = MyProblem('Arad', 'Bucharest', romania_map)

    print("Breadth First Search:")
    print("Nodes visited:")
    bfs = breadth_first_graph_search(romania_problem)
    print("Optimal way:")
    print(bfs.solution())
    #print("Path cost: {0}".format(bfs.path_cost()))

    print("\nDepth First Search")
    print("Nodes visited:")
    dfs = depth_first_graph_search(romania_problem)
    print("Optimal way:")
    print(dfs.solution())

    print("\nUniform Cost Search")
    print("Nodes visited:")
    ucs = uniform_cost_search(romania_problem)
    print("Optimal way:")
    print(ucs.solution())

    print("\nBest First Search")
    print("Nodes visited:")
    bestfs = best_first_graph_search(romania_problem, lambda node: node.state)
    print("Optimal way:")
    print(bestfs.solution())

    print("\nA* Search:")
    print("Nodes visited:")
    astar = astar_search(romania_problem)
    print("Optimal way:")
    print(astar.solution())



if __name__ == '__main__':
    main()