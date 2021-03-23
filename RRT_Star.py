import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import graph_generated, printNode, compareNodes, getSameNode, checkinThis
from maps_utils import checkInObstacle, DistanceBetween, \
    map_size, pointEncompassed, path_colour, Node
from utils import GenerateVideo, generateRandomPoint
import time
import math
from math import sqrt, cos, sin, atan2


class RRTStar:
    """
    RRTStar Class
    """

    def __init__(self, graph, start_node, goal_node):
        """
        RRTStar init function
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
        self.nodes = []
        self.goal_reached = False
        self.iterations = 2000
        self.threshold = 9
        self.video_count = 0
        # Exclusive param for RRTStar.
        # Tune the radius parameter accordingly
        self.radius = 15

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

    def findClosestNode(self, point_1, point_2):
        """
        Fucntion to find the closest node that is
        within the threshold
        :param point_1: Point 1
        :type point_1: Node
        :param point_2: Point 2
        :type point_2: Node
        :return: Correct Node
        :rtype: Node
        """
        if DistanceBetween(point_1, point_2) < self.threshold:
            return point_2
        else:
            theta = atan2(point_2.y - point_1.y, point_2.x - point_1.x)
            close_node = Node(int(point_1.x + self.threshold * cos(theta)),
                              int(point_1.y + self.threshold * sin(theta)))
            return close_node

    # Added for RRTStar

    def chooseNewParent(self, current_nearest_node, new_node):
        """
        Function to find a new node with a lower cost to go to.
        This now becomes a newer nearest node, nn.
        :param current_nearest_node: Current Nearest Node
        :type current_nearest_node: Nde
        :param new_node: New Interpolated Node
        :type new_node: Node
        :return: New Node, and a Nearest Node
        :rtype: Node, Node
        """

        graph_vertices = self.graph.getVertices()

        # Loops through all the node in the self.nodes list
        # Hence this method gets slower as more and more nodes
        # get added to the list

        for p in self.nodes:

            if DistanceBetween(p, new_node) < self.radius and \
                    p.cost + DistanceBetween(p, new_node) < \
                    current_nearest_node.cost + DistanceBetween(current_nearest_node, new_node):
                print('Nearest node changed')
                current_nearest_node = p

        # New interpolated nodes cost is changed

        new_node.cost = current_nearest_node.cost + \
                        DistanceBetween(current_nearest_node, new_node)

        new_node = getSameNode(new_node, graph_vertices)
        current_nearest_node = getSameNode(current_nearest_node, graph_vertices)

        # That nodes parent is set as the current
        # nearest node

        new_node.parent = current_nearest_node

        return new_node, current_nearest_node

    # Added for RRTStar

    def reWire(self, new_node):
        """
        Function exclusive to RRT Star to rewire the path
        based on the new interpolated node, received from
        the choose parent function

        :param new_node: New interpolated Node
        :type new_node: Node
        :return: None
        :rtype: None
        """
        graph_vertices = self.graph.getVertices()

        for i in range(len(self.nodes)):
            p = self.nodes[i]

            # A check is done now to make sure that for every
            # node in the node list, if the cost will
            # decrease when rewired. If this happens the
            # rewiring is done and the new vertex is replaced
            # in the nodes list

            if not compareNodes(p, new_node.parent) \
                    and DistanceBetween(p, new_node) < self.radius \
                    and new_node.cost + DistanceBetween(p, new_node) < p.cost:
                p.parent = new_node
                p.cost = new_node.cost + DistanceBetween(p, new_node)
                p = getSameNode(p, graph_vertices)

                self.nodes[i] = p

    def SolveRRTStar(self, starting_vertex, goal_vertex):
        """
        Solves RRTStar
        :param starting_vertex: Starting Node
        :type starting_vertex: Node
        :param goal_vertex: Goal Node
        :type goal_vertex: Node
        :return: None
        :rtype: None
        """

        self.nodes.append(starting_vertex)
        graph_vertices = self.graph.getVertices()

        path_dic = {}
        for i in range(self.iterations):

            random_generated_node = generateRandomPoint(map_size, map_canvas)
            random_generated_node = getSameNode(random_generated_node, graph_vertices)

            # nearest node
            nn = self.nodes[0]

            for p in self.nodes:
                if DistanceBetween(p, random_generated_node) < DistanceBetween(random_generated_node, nn):
                    nn = p

            nn = getSameNode(nn, graph_vertices)

            new_node = self.findClosestNode(nn, random_generated_node)
            new_node = getSameNode(new_node, graph_vertices)

            # Choose the new parent
            new_node, nn = self.chooseNewParent(nn, new_node)

            # Adding the node to the nodes list
            self.nodes.append(new_node)

            # rewire with this new node in mind
            self.reWire(new_node)

            path_dic[nn] = new_node
            self.nodes.append(new_node)

            cv2.line(map_canvas, (new_node.x, new_node.y),
                     (nn.x, nn.y), path_colour, 1, cv2.LINE_AA)

            cv2.imshow("Searching map", map_canvas)

            # To save the video
            len_number = len(str(self.video_count))
            number_name = "0" * (6 - len_number)
            cv2.imwrite('RRTStar_Video_Images/' + number_name + str(self.video_count) + '.jpg', map_canvas)
            self.video_count += 1

            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

            if pointEncompassed(new_node, goal_vertex):
                print(' - - - GOAL FOUND - - - ')
                self.goal_reached = True
                print('Video Generating ....')
                break

            if i == self.iterations - 1:
                print('Sorry, Node not found. Try Tuning the parameters')
                break


def doRRTStar():
    """
    RRTStar Function to be executed when
    Tkinter Button is clicked
    """
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    time_s_tk = time.time()
    rrt_tk = RRTStar(graph_generated, node_start, node_goal)
    rrt_tk.SolveRRTStar(node_start, node_goal)
    print('Total Time for execution : ', time.time() - time_s_tk, ' seconds')
    image_folder_name = "RRTStar_Video_Images"
    file = "RRTStar_Video"
    GenerateVideo(image_folder_name, file, video_folder="Videos")


if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    time_s = time.time()
    rrt = RRTStar(graph_generated, clicked_start, clicked_goal)
    rrt.SolveRRTStar(clicked_start, clicked_goal)
    print('Total Time for execution : ', time.time() - time_s, ' seconds')
    image_folder = "RRTStar_Video_Images"
    file_name = "RRTStar_Video"
    GenerateVideo(image_folder, file_name, video_folder="Videos")
