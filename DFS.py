import cv2
from graph import graph_generated, checkinThis, printNode
from map import map_canvas, mouse_start_node, mouse_goal_node
from maps_utils import resolution, pointEncompassed, visited_colour
from utils import GenerateVideo
import time


# Depth First Search Class

class DepthFirstSearch:
    """
    This class describes a depth first search.
    """

    def __init__(self, graph, start_node, goal_node):

        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = []
        self.goal_reached = False
        self.count = 0

    def solveDepthFirstSearch(self, node):
        """
        Solves the graph using DFS Algorithm
        
        :param      node:  The current node
        :type       node:  Node type
        """

        video_count = 0

        # Checking if the goal is within the radius
        if pointEncompassed(node, self.goal_node):
            print('Length of Visited :', len(self.visited))
            print(' - - - Goal Reached - - - ')
            self.goal_reached = True
            print('Video Generating ....')

        graph_keys = list(self.graph.getVertices())
        if len(graph_keys) == len(self.visited):
            return None
        else:
            self.visited.append(node)
            cv2.circle(map_canvas, (node.x, node.y), resolution, visited_colour, -1, cv2.LINE_AA)
            # To save the Video
            len_number = len(str(video_count))

            number_name = "0" * (6 - len_number)
            cv2.imwrite('DFS_Video_Images/' + number_name + str(video_count) + '.jpg', map_canvas)
            video_count += 1

            neighbours = self.graph.getNeighbors(node)

            if neighbours is not None:
                for neighbour in neighbours:
                    self.count += 1
                    cv2.imshow("Searching map", map_canvas)
                    if cv2.waitKey(20) & 0xFF == ord('q'):
                        break
                    if not checkinThis(neighbour, self.visited):
                        if not self.goal_reached:
                            self.solveDepthFirstSearch(neighbour)


def doDFS():
    """
    BFS Function to be executed when
    Tkinter Button is clicked
    """
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    time_s_tk = time.time()
    dfs_tk = DepthFirstSearch(graph_generated, node_start, node_goal)
    dfs_tk.solveDepthFirstSearch(node_start)
    print('Total Time for execution : ', time.time() - time_s_tk, ' seconds')
    image_folder_name = "DFS_Video_Images"
    file = "DFS_Video"
    GenerateVideo(image_folder_name, file, video_folder="Videos")


# Main function to run the DFS
if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    time_s = time.time()
    dfs = DepthFirstSearch(graph_generated, clicked_start, clicked_goal)
    dfs.solveDepthFirstSearch(clicked_start)
    print('Total Time for execution : ', time.time() - time_s, ' seconds')
    image_folder = "DFS_Video_Images"
    file_name = "DFS_Video"
    GenerateVideo(image_folder, file_name, video_folder="Videos")
