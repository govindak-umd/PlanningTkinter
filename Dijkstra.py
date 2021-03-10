import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import graph_generated, checkinThis, printNode
from maps_utils import resolution, path_colour, pointEncompassed, visited_colour


class Queue:

    def __init__(self):
