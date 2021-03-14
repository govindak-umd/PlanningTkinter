import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, cost_graph_generated, checkinThis, printNode
from maps_utils import Node, resolution, pointEncompassed, visited_colour
from data_structures import PriorityQueue
from utils import GenerateVideo
import time


def EuclideanHeuristic(start_node, goal_node):
    """
    Function to calculate the Euclidean Distance
    :param start_node: start node
    :type start_node: Node
    :param goal_node: Goal Node
    :type goal_node: Node
    :return: Euclidean distance units
    :rtype: Integer
    """
    x_1 = start_node.x
    y_1 = start_node.y

    x_2 = goal_node.x
    y_2 = goal_node.y

    euc_distance = ((x_2 - x_1) ** 2 + (y_2 - y_1) ** 2) ** 0.5

    return euc_distance


def ManhattanHeuristic(start_node, goal_node):
    """
    Function to calculate the Manhattan Distance
    :param start_node: start node
    :type start_node: Node
    :param goal_node: Goal Node
    :type goal_node: Node
    :return: Manhattan distance units
    :rtype: Integer
    """
    x_1 = start_node.x
    y_1 = start_node.y

    x_2 = goal_node.x
    y_2 = goal_node.y

    manhattan_distance = abs((x_2 - x_1)) + abs((y_2 - y_1))

    return manhattan_distance


def A_Star_Solve(graph, starting_vertex, goal_vertex):
    """
    Solve A star Path Planning algorithm
    :param graph: Graph of the map
    :type graph: Graph Type
    :param starting_vertex: Starting Node
    :type starting_vertex: Node
    :param goal_vertex: Goal Node
    :type goal_vertex: Node
    """
    # f = g + h
    # g is the distance between the current node and the start node
    # h is the distance between the current node adn the goal node (heuristic)

    video_count = 0
    goal_reached = 0

    # Returns the vertices of the graph
    graph_vertices = graph.getVertices()

    starting_vertex = getSameNode(starting_vertex, graph_vertices)

    distances = {vertex: float('infinity') for vertex in graph_vertices}
    distances[starting_vertex] = 0

    starting_vertex = getSameNode(starting_vertex, graph_vertices)

    # Initializes the Priority Queue, which is our open list
    open_list = PriorityQueue()

    # Adding start node to the open list
    open_list.insert_pq(0, starting_vertex)

    closed_list = []

    print('Starting Node')
    printNode(starting_vertex)

    # Runs while loop until goal node is not found
    while open_list.len_pq() > 0 and goal_reached == 0:

        # Gets the node as per the priority queue
        current_distance, current_vertex = open_list.pop_pq()
        print('current_vertex')
        printNode(current_vertex)
        print('Cost : ', current_distance)

        # draws the circle
        cv2.circle(map_canvas, (current_vertex.x, current_vertex.y), resolution, visited_colour, -1,
                   cv2.LINE_AA)

        # To save the Video
        len_number = len(str(video_count))
        number_name = "0" * (6 - len_number)
        cv2.imwrite('A_Star_Video_Images/' + number_name + str(video_count) + '.jpg', map_canvas)
        video_count += 1

        closed_list.append(current_vertex)

        # Gets the equivalent node from the graph
        current_vertex = getSameNode(current_vertex, graph_vertices)

        # Checks if the goal is within the radius specified
        # in the utils file
        if pointEncompassed(current_vertex, goal_vertex):
            print(' - - - GOAL FOUND - - - ')
            # Sets the value to True
            goal_reached = 1
            print('Video Generating ....')

        # A dictionary containing the neighbours and the weight/cost to reach them
        neighbours_dictionary = cost_graph_generated.getNeighbors(current_vertex).items()
        print(' ----- Neighbours -----  ')
        for neighbour, weight in neighbours_dictionary:
            print('neighbour ... ')
            printNode(neighbour)

            # Gets the equivalent node from the graph
            neighbour = getSameNode(neighbour, graph_vertices)

            # Checking if the neighbour is in the closed list
            if checkinThis(neighbour, closed_list):
                continue

            # Adds the weight to the distance to reach the current node so far
            neighbour_g_cost = weight
            # h_cost = 0, for Dijkstra,
            # Can have an Euclidean, or a Manhattan Heuristic
            neighbour_h_cost = EuclideanHeuristic(neighbour, goal_vertex)
            # f_cost is the sum of h_cost and g_cost
            neighbour_f_cost = neighbour_g_cost + neighbour_h_cost
            print('distances[neighbour] :', distances[neighbour], '| neighbour_g_cost : ', neighbour_g_cost,
                  '| neighbour_h_cost : ', neighbour_h_cost, '| neighbour_f_cost : ', neighbour_f_cost)
            # If the distance to the node is less than the
            # previously stored distance to that neighbour,

            if open_list.checkinPQ(neighbour):
                if neighbour_g_cost < distances[neighbour]:


                    # print('Distance so far : ', distance)
                    # Replace the distance value

                    distances[neighbour] = neighbour_f_cost

                    # Shows the traversal on map
                    cv2.imshow("Searching map", map_canvas)

                    if cv2.waitKey(20) & 0xFF == ord('q'):
                        break

                    # Inserts the new node back into the priority queue as
                    # per the rules of the Priority Queue Class.
                    # This new neighbour will be the one that can be traversed
                    # to with the lowest cost

            open_list.insert_pq(neighbour_f_cost, neighbour)
        print('------------------')


def doAStarPathPlanning():
    """
    A * Function to be executed when
    Tkinter Button is clicked
    """
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    time_s_tk = time.time()
    A_Star_Solve(cost_graph_generated, node_start, node_goal)
    print('Total Time for execution : ', time.time() - time_s_tk, ' seconds')
    image_folder_name = "A_Star_Video_Images"
    file_name = "A_star_Video"
    GenerateVideo(image_folder_name, file_name, video_folder="Videos")


# Main function to run A Star
if __name__ == "__main__":
    clicked_start = mouse_start_node
    clicked_goal = mouse_goal_node
    # clicked_start = Node(200,50)
    # clicked_goal = Node(50,200)
    time_s = time.time()
    # Run the A Star Solve Function
    A_Star_Solve(cost_graph_generated, clicked_start, clicked_goal)
    print('Total Time for execution : ', time.time() - time_s, ' seconds')
    image_folder = "A_Star_Video_Images"
    file = "A_star_Video"
    GenerateVideo(image_folder, file, video_folder="Videos")
