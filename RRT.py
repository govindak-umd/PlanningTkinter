import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, cost_graph_generated, checkinThis, printNode
from maps_utils import DistanceBetween, border_size, DistanceBetween, map_size, Node, resolution, pointEncompassed, \
    visited_colour
from data_structures import PriorityQueue
from utils import GenerateVideo
import time
import random
import numpy as np

rrt_min_distance = 1
rrt_num_nodes = 2000
rrt_thresh = 3
rrt_goal_radius = resolution


# Check if a point is in an obstacle
def checkinObstacle(node_check):
    if map_canvas[node_check.x, node_check.y][0] == 0:
        return True
    return False


def RRT_ToNextNode(node_from_list, node_rand):
    global rrt_thresh
    if DistanceBetween(node_from_list, node_rand) < rrt_thresh:
        return node_rand
    else:
        pass


# Generate a random node for adding to tree for traversal
def RRT_generateRandomPoint():
    while True:
        x_rand = int(random.random() * map_size)
        y_rand = int(random.random() * map_size)
        node_random = Node(x_rand, y_rand)
        if checkinObstacle(node_random):
            continue
        return node_random


def SolveRRT(starting_vertex, goal_vertex):
    nodes = []
    nodes_traversed = 0
    global rrt_min_distance
    global rrt_num_nodes
    global rrt_thresh
    global rrt_goal_radius

    while True:
        nodes_traversed += 1
        if nodes_traversed < rrt_num_nodes:
            foundNextNode = False
            while not foundNextNode:
                current_rand_node = RRT_generateRandomPoint()
                curr_parent = nodes[0]
                for n in nodes:
                    if DistanceBetween(n, current_rand_node) < DistanceBetween(curr_parent, current_rand_node):
                        new_n = RRT_ToNextNode(n, current_rand_node)
                        if not checkinObstacle(new_n):
                            curr_parent = new_n
                            foundNextNode = True

                if pointEncompassed(current_rand_node, goal_vertex):
                    print(' - - - GOAL FOUND - - - ')
                    # Sets the value to True
                    goal_reached = 1
                    print('Video Generating ....')


if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    # SolveRRT(clicked_start, clicked_goal)
    # Checks if the goal is within the radius specified
    # in the utils file
