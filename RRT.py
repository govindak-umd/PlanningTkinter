import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import graph_generated, printNode, compareNodes, getSameNode, checkinThis
from maps_utils import checkInObstacle, DistanceBetween, \
    map_size, Node, pointEncompassed, path_colour
from utils import GenerateVideo, generateRandomPoint
import time
import math
from math import sqrt, cos, sin, atan2


class RRT:
    """
    RRT Class
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
        self.iterations = 2000
        self.threshold = 9
        self.video_count = 0

    def checkPathCollision(self, node_from, node_to):
        # check if the path between node_from and
        # node_end is collision free
        pass

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
            from_x, from_y = node_from.x, node_from.y
            to_x, to_y = node_to.x, node_to.y
            theta = math.atan2(to_y - from_y, to_x - from_x)
            node_x = int(from_x + self.threshold * math.cos(theta))
            node_y = int(from_y + self.threshold * math.sin(theta))
            new_point = Node(node_x, node_y)

            return new_point

    def BackTracking(self, tree_dic, goal):
        """
        Function to get the BackTracking path
        :param tree_dic: Tree Dictionary
        :type tree_dic: Dictionary
        """
        curr_child = goal
        for _ in range(1000):
            for k, v in tree_dic.items():
                if compareNodes(v, curr_child):
                    cv2.line(map_canvas, (curr_child.x, curr_child.y),
                             (k.x, k.y), [0, 0, 255], 1, cv2.LINE_AA)
                    curr_child = k
                    cv2.imwrite('Final_RRT.jpg', map_canvas)

    def step_from_to(self, point_1, point_2):
        if DistanceBetween(point_1, point_2) < self.threshold:
            return point_2
        else:
            theta = atan2(point_2.y - point_1.y, point_2.x - point_1.x)
            close_node = Node(int(point_1.x + self.threshold * cos(theta)),
                              int(point_1.y + self.threshold * sin(theta)))
            return close_node

    def SolveRRT(self, starting_vertex, goal_vertex):

        nodes = [starting_vertex]
        graph_vertices = self.graph.getVertices()

        path_dic = {}
        for i in range(self.iterations):

            random_generated_node = generateRandomPoint(map_size, map_canvas)
            random_generated_node = getSameNode(random_generated_node, graph_vertices)

            nn = nodes[0]

            for p in nodes:
                if DistanceBetween(p, random_generated_node) < DistanceBetween(random_generated_node, nn):
                    nn = p

            new_node = self.step_from_to(nn, random_generated_node)
            new_node = getSameNode(new_node, graph_vertices)

            path_dic[nn] = new_node
            nodes.append(new_node)

            cv2.line(map_canvas, (new_node.x, new_node.y),
                     (nn.x, nn.y), path_colour, 1, cv2.LINE_AA)

            cv2.imshow("Searching map", map_canvas)

            # To save the video
            len_number = len(str(self.video_count))
            number_name = "0" * (6 - len_number)
            # cv2.imwrite('RRT_Video_Images/' + number_name + str(self.video_count) + '.jpg', map_canvas)
            self.video_count += 1

            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

            if pointEncompassed(new_node, goal_vertex):
                print(' - - - GOAL FOUND - - - ')
                self.goal_reached = True
                self.BackTracking(path_dic, new_node)
                print('Video Generating ....')
                break

            if i == self.iterations - 1:
                print('Sorry, Node not found. Try Tuning the parameters')
                break


def doRRT():
    """
    RRT Function to be executed when
    Tkinter Button is clicked
    """
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    time_s_tk = time.time()
    rrt_tk = RRT(graph_generated, node_start, node_goal)
    rrt_tk.SolveRRT(node_start, node_goal)
    print('Total Time for execution : ', time.time() - time_s_tk, ' seconds')
    image_folder_name = "RRT_Video_Images"
    file = "RRT_Video"
    GenerateVideo(image_folder_name, file, video_folder="Videos")


if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    time_s = time.time()
    rrt = RRT(graph_generated, clicked_start, clicked_goal)
    rrt.SolveRRT(clicked_start, clicked_goal)
    print('Total Time for execution : ', time.time() - time_s, ' seconds')
    image_folder = "RRT_Video_Images"
    file_name = "RRT_Video"
    # GenerateVideo(image_folder, file_name, video_folder="Videos")
