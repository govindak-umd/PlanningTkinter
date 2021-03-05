import cv2
from map import map_canvas
from graph import graph_generated
from graph import checkinThis
from graph import compareNodes
from map import mouse_start_node
from map import mouse_goal_node
from graph import printNode
from maps_utils import resolution
from maps_utils import path_colour
from maps_utils import pointEncompassed


class DFS:

    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = []
        self.goal_reached = False

    def solveDFS(self, start):
        print('Searching ... ')
        printNode(start)
        # Make sure the goal is within the circle being tested
        # if compareNodes(start, self.goal_node):
        #     print('Length of Visited :', len(self.visited))
        #     print(' - - - Goal Reached - - - ')
        #     self.goal_reached = True
        if pointEncompassed(start, self.goal_node):
            print('Length of Visited :', len(self.visited))
            print(' - - - Goal Reached - - - ')
            self.goal_reached = True
        graph_keys = list(self.graph.getVertices())
        if len(graph_keys) == len(self.visited):
            return None
        else:
            self.visited.append(start)
            neighbours = self.graph.getNeighbors(start)
            if neighbours is not None:
                for neighbour in neighbours:
                    # Change color of the path
                    cv2.circle(map_canvas, (neighbour.y, neighbour.x), resolution, path_colour, -1, cv2.LINE_AA)
                    cv2.imshow("Searching map", map_canvas)
                    if cv2.waitKey(20) & 0xFF == ord('q'):
                        break
                    if not checkinThis(neighbour, self.visited):
                        if not self.goal_reached:
                            self.solveDFS(neighbour)


if __name__ == "__main__":
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    dfs = DFS(graph_generated, node_start, node_goal)
    dfs.solveDFS(node_start)
