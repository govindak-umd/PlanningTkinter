import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, cost_graph_generated, compareNodes, graph_generated, checkinThis, printNode
from maps_utils import resolution, pointEncompassed, visited_colour, path_colour
from data_structures import PriorityQueue

def EuclideanHeuristic(start_node, goal_node):
    x_1 = start_node.x
    y_1 = start_node.y

    x_2 = goal_node.x
    y_2 = goal_node.y

    euc_distance = ((x_2 - x_1) ** 2 + (y_2 - y_1) ** 2) ** 0.5

    return euc_distance


def ManhattanHeuristic(start_node, goal_node):
    x_1 = start_node.x
    y_1 = start_node.y

    x_2 = goal_node.x
    y_2 = goal_node.y

    manhattan_distance = abs((x_2 - x_1)) + abs((y_2 - y_1))

    return manhattan_distance

