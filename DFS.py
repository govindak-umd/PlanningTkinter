import cv2
from maps_utils import map_size
from maps_utils import border_size
import numpy as np
from map import map_canvas
import time
from graph import graph_generated
from graph import Node
from graph import checkinThis
from graph import checkinGraph
from graph import printNode
from graph import getSameNode
from graph import compareNodes


class DFS:

    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = []
        self.goal_reached = False

    def solveDFS(self, start):
        print('Searching ... ')
        if compareNodes(start, self.goal_node):
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
                    # Change color of the frame
                    map_canvas[neighbour.x, neighbour.y] = (200, 25, 25)
                    cv2.imshow("Searching map", map_canvas)
                    if cv2.waitKey(20) & 0xFF == ord('q'):
                        break
                    if not checkinThis(neighbour, self.visited):
                        if not self.goal_reached:
                            self.solveDFS(neighbour)


if __name__ == "__main__":
    node_start = Node(200, 15)
    node_goal = Node(234, 400)
    dfs = DFS(graph_generated, node_start, node_goal)
    dfs.solveDFS(node_start)
