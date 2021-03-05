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


class DFS:

    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = set()


if __name__ == "__main__":
    node_start = Node(20, 15)
    node_goal = Node(20, 17)
    dfs = DFS(graph_generated, node_start, node_goal)