import cv2
from maps_utils import map_size
from maps_utils import border_size
import numpy as np
from map import map_canvas
import time
from graph import graph_generated
from graph import Node
from graph import compareNodes


# DFS Class
class DFS:
    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node

    def startSolving(self):
        visited = set()

        vertices = self.graph.getVertices()
        for node in vertices:
            if compareNodes(node, node_start):
                print('Node found')


if __name__ == "__main__":
    node_start = Node(20, 15)
    node_goal = Node(250, 150)
    dfs = DFS(graph_generated, node_start, node_goal)
    dfs.startSolving()
