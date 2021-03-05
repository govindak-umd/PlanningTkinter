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


# DFS Class
class DFS:
    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = set()

    def startSolving(self):
        key_list = self.graph.getVertices()
        # Declaring a set to save the visited nodes
        if not checkinGraph(self.start_node, self.graph):
            print('That is not a valid start node')

        if not checkinGraph(self.goal_node, self.graph):
            print('That is not a valid goal node')

        backtrack_path = []
        stack_dfs = [self.start_node]
        while len(stack_dfs) != 0:
            node_popped = stack_dfs.pop()
            print(node_popped)
            if not checkinThis(node_popped, backtrack_path):
                backtrack_path.append(node_popped)
            if not checkinGraph(node_popped, self.graph):
                continue
            node_popped2 = getSameNode(node_popped, key_list)
            neighbours = self.graph.getNeighbors(node_popped2)
            for neighbour in neighbours:
                stack_dfs.append(neighbour)


if __name__ == "__main__":
    node_start = Node(20, 15)
    printNode(node_start)
    graph_generated.getNeighbors(node_start)
    node_goal = Node(20, 150)
    dfs = DFS(graph_generated, node_start, node_goal)
    dfs.startSolving()
