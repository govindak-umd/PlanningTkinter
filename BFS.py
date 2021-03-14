import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import graph_generated, checkinThis, printNode
from maps_utils import resolution, pointEncompassed, visited_colour
from utils import GenerateVideo


# Breadth First Search Class
class BreadthFirstSearch:
    """
    This class describes a depth first search.
    """

    def __init__(self, graph, start_node, goal_node):

        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = []
        self.queue = []
        self.goal_reached = False
        self.count = 0

    def solveBreadthFirstSearch(self, node):
        """
        Solves the graph using BFS Algorithm

        :param      node:  The current node
        :type       node:  Node type
        """
        self.visited.append(node)
        self.queue.append(node)
        video_count = 0

        while self.queue and self.goal_reached is False:

            queue_lifo = self.queue.pop(0)
            neighbours = self.graph.getNeighbors(queue_lifo)
            if neighbours is not None:
                for neighbour in neighbours:
                    if not checkinThis(neighbour, self.visited):
                        self.visited.append(neighbour)
                        self.queue.append(neighbour)

                        # Checking if the goal is within the radius
                        if pointEncompassed(neighbour, self.goal_node):
                            print('Length of Visited :', len(self.visited))
                            print('Length of Queue :', len(self.queue))
                            print(' - - - Goal Reached - - - ')
                            self.goal_reached = True
                        cv2.circle(map_canvas, (neighbour.x, neighbour.y), resolution, visited_colour, -1, cv2.LINE_AA)

                        # To save the Video
                        len_number = len(str(video_count))
                        number_name = "0" * (6 - len_number)
                        cv2.imwrite('BFS_Video_Images/' + number_name + str(video_count) + '.jpg', map_canvas)
                        video_count += 1

                        cv2.imshow("Searching map", map_canvas)
                        if cv2.waitKey(20) & 0xFF == ord('q'):
                            break


def doBFS():
    """
    BFS Function to be executed when
    Tkinter Button is clicked
    """
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    bfs_tk = BreadthFirstSearch(graph_generated, node_start, node_goal)
    bfs_tk.solveBreadthFirstSearch(node_start)
    image_folder_name = "BFS_Video_Images"
    file = "BFS_Video"
    GenerateVideo(image_folder_name, file, video_folder="Videos")


# Main function to run the BFS
if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    bfs = BreadthFirstSearch(graph_generated, clicked_start, clicked_goal)
    bfs.solveBreadthFirstSearch(clicked_start)
    image_folder = "BFS_Video_Images"
    file_name = "BFS_Video"
    GenerateVideo(image_folder, file_name, video_folder="Videos")
