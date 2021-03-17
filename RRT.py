import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, graph_generated, checkinThis, printNode
from maps_utils import DistanceBetween, border_size, DistanceBetween, map_size, Node, resolution, pointEncompassed, \
    visited_colour, path_colour
from data_structures import PriorityQueue
from utils import GenerateVideo
import time
import random
import numpy as np


class RRT:

    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = set()
        self.path = []
        self.goal_reached = False
        self.count = 0
        self.iterations = 2000
        self.threshold = 3

    # Check if a point is in an obstacle
    def checkinObstacle(self, node_check):
        if map_canvas[node_check.x, node_check.y][0] == 0:
            return True
        return False

    def RRT_ToNextNode(self, node_from_list, node_rand):
        if DistanceBetween(node_from_list, node_rand) < self.threshold:
            return node_rand
        else:
            return node_from_list

    # Generate a random node for adding to tree for traversal
    def RRT_generateRandomPoint(self):
        while True:
            x_rand = int(random.random() * map_size)
            y_rand = int(random.random() * map_size)
            node_random = Node(x_rand, y_rand)
            if self.checkinObstacle(node_random):
                continue
            return node_random

    def findClosestPointInTree(self, node, vertices):
        pass

    def SolveRRT(self, starting_vertex, goal_vertex):
        self.visited.add(starting_vertex)
        for i in range(self.iterations-1000):
            random_generated_node = self.RRT_generateRandomPoint()
            printNode(random_generated_node)

        # while True:
        #     nodes_traversed += 1
        #     if nodes_traversed < rrt_num_nodes:
        #         foundNextNode = False
        #         while not foundNextNode and goal_reached == 0:
        #             nodes.pop(0)
        #             print('len of nodes : ', len(nodes))
        #             current_rand_node = RRT_generateRandomPoint()
        #             # draws the circle
        #             cv2.circle(map_canvas, (current_rand_node.x, current_rand_node.y), resolution, visited_colour, -1,
        #                        cv2.LINE_AA)
        #
        #             curr_parent = nodes[0]
        #
        #             print('curr_parent : ', curr_parent)
        #
        #             for n in nodes:
        #                 if DistanceBetween(n, current_rand_node) < DistanceBetween(curr_parent, current_rand_node):
        #                     new_n = RRT_ToNextNode(n, current_rand_node)
        #                     if not checkinObstacle(new_n):
        #                         curr_parent = new_n
        #                         foundNextNode = True
        #
        #             if pointEncompassed(current_rand_node, goal_vertex):
        #                 print(' - - - GOAL FOUND - - - ')
        #                 # Sets the value to True
        #                 goal_reached = 1
        #                 print('Video Generating ....')
        #
        #             nodes.append(curr_parent)
        #             cv2.line(map_canvas, (current_rand_node.x, current_rand_node.y), (curr_parent.x, curr_parent.y),
        #                      path_colour, 1, cv2.LINE_AA)
        #             # Shows the traversal on map
        #             cv2.imshow("Searching map", map_canvas)
        #
        #             if cv2.waitKey(20) & 0xFF == ord('q'):
        #                 break


if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    rrt = RRT(graph_generated, clicked_start, clicked_goal)
    rrt.SolveRRT(clicked_start, clicked_goal)
    # Checks if the goal is within the radius specified
    # in the utils file
