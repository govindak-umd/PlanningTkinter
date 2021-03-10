import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import cost_graph_generated, graph_generated, checkinThis, printNode
from maps_utils import resolution, path_colour, pointEncompassed, visited_colour
import heapq


def DijkstraSolve(dijkstra_graph, start_node):
    pass


# Main function to run Dijkstra
if __name__ == "__main__":
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    DijkstraSolve(cost_graph_generated, node_start)
