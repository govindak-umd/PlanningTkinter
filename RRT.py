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
import math


class RRT:

    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.vertices = set()
        self.path = []
        self.goal_reached = False
        self.count = 0
        self.iterations = 2000
        self.threshold = 5
        self.video_count = 0
        self.parent_dic = {}

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

    def findClosestPointInTree(self, node):
        min_dist = float('inf')
        print('Point to check in vertices of length > ', len(self.vertices))
        printNode(node)
        for vertex in self.vertices:
            calc_distance = DistanceBetween(vertex, node)
            if calc_distance < min_dist:
                min_dist = calc_distance
                closest_tree_node = vertex
        print('Closest Point : ')
        printNode(closest_tree_node)
        return closest_tree_node

    def checkPathCollision(self, node_from, node_to):
        # check if the path between node_from and
        # node_end is collision free
        pass

    def getPointWithinThreshold(self, node_from, node_to):

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

    def setParent(self, parent_point, child_point):
        self.parent_dic[child_point] = parent_point

    def NoCollisionDetected(self, node_from, node_end):

        pass

    def SolveRRT(self, starting_vertex, goal_vertex):
        self.vertices.add(starting_vertex)

        for i in range(self.iterations):
            random_generated_node = self.RRT_generateRandomPoint()
            closest_node_to_random_point = self.findClosestPointInTree(random_generated_node)
            new_point = self.getPointWithinThreshold(closest_node_to_random_point, random_generated_node)

            # Ideally add a code here to make sure that the
            # straight path between the two points won't collide

            self.vertices.add(new_point)

            self.setParent(closest_node_to_random_point, new_point)

            cv2.line(map_canvas, (closest_node_to_random_point.x, closest_node_to_random_point.y),
                     (new_point.x, new_point.y), path_colour, 1, cv2.LINE_AA)

            cv2.imshow("Searching map", map_canvas)

            # To save the video
            len_number = len(str(self.video_count))
            number_name = "0" * (6 - len_number)
            cv2.imwrite('RRT_Video_Images/' + number_name + str(self.video_count) + '.jpg', map_canvas)
            self.video_count+=1

            if cv2.waitKey(20) & 0xFF == ord('q'):
                break
            if pointEncompassed(new_point, goal_vertex):
                print(' - - - GOAL FOUND - - - ')
                # Sets the value to True
                self.goal_reached = True
                print('Video Generating ....')
                break


if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    time_s = time.time()
    rrt = RRT(graph_generated, clicked_start, clicked_goal)
    rrt.SolveRRT(clicked_start, clicked_goal)
    print('Total Time for execution : ', time.time() - time_s, ' seconds')
    image_folder = "RRT_Video_Images"
    file_name = "RRT_Video"
    GenerateVideo(image_folder, file_name, video_folder="Videos")
