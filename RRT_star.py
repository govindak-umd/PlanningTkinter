import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import graph_generated, printNode
from maps_utils import checkInObstacle, DistanceBetween, \
    map_size, Node, pointEncompassed, path_colour
from utils import GenerateVideo, generateRandomPoint
import time
import random
import math


class RRTStar:
    """
    RRT Star Class
    """

    def __init__(self, graph, start_node, goal_node):
        """
        RRT init function
        :param graph: The graph which contains all the nodes
        :type graph: Graph Type
        :param      start_node:  The starting vertex
        :type       start_node:  Node type
        :param      goal_node:   The goal vertex
        :type       goal_node:   Node type
        """
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.vertices = set()
        self.path = []
        self.goal_reached = False
        self.count = 0
        self.iterations = 4000
        self.threshold = 5
        self.video_count = 0
        self.parent_dic = {}

    def getPointWithinThreshold(self, node_from, node_to):
        """
        Function to make sure the random node is
        within the threshold distance away from the parent
        :param node_from: The previous parent
        :type node_from: Node
        :param node_to: The random node
        :type node_to: Node
        :return: The closer node
        :rtype: Node
        """
        if DistanceBetween(node_from, node_to) < self.threshold:

            return node_to
        else:
            print('Point too far ... ')
            from_x, from_y = node_from.x, node_from.y
            to_x, to_y = node_to.x, node_to.y
            theta = math.atan2(to_y - from_y, to_x - from_x)
            node_x = int(from_x + self.threshold * math.cos(theta))
            node_y = int(from_y + self.threshold * math.sin(theta))
            new_point = Node(node_x, node_y)

            return new_point

    def SolveRRTStar(self, starting_vertex, goal_vertex):
        """
        Solve the graph from start to end through
        Rapidly Exploring Random Trees * (RRT * )
        :param starting_vertex: Starting Node
        :type starting_vertex: Node
        :param goal_vertex: Goal Node
        :type goal_vertex: Node
        """
        pass


def doRRT_Star():
    """
    RRT * Function to be executed when
    Tkinter Button is clicked
    """
    pass


if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    time_s = time.time()
    rrt_star = RRTStar(graph_generated, clicked_start, clicked_goal)
    rrt_star.SolveRRT(clicked_start, clicked_goal)
    print('Total Time for execution : ', time.time() - time_s, ' seconds')
    image_folder = "RRT_Star_Video_Images"
    file_name = "RRT_Star_Video"
