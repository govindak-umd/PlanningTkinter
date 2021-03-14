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


def A_Star_Solve(graph, starting_vertex, goal_vertex):
    # f = g + h
    # g is the distance between the current node and the start node
    # h is the distance between the current node adn the goal node (heuristic)

    goal_reached = 0

    # Returns the vertices of the graph
    graph_vertices = graph.getVertices()

    distances = {vertex: float('infinity') for vertex in graph_vertices}
    distances[starting_vertex] = 0

    starting_vertex = getSameNode(starting_vertex, graph_vertices)

    # Initializes the Priority Queue, which is our open list
    priority_queue = PriorityQueue()

    # Adding start node to the open list
    priority_queue.insert_pq(0, starting_vertex)

    closed_list = []

    # Runs while loop until goal node is not found
    while priority_queue.len_pq() > 0 and goal_reached == 0:

        # Gets the node as per the priority queue
        current_distance, current_vertex = priority_queue.pop_pq()

        closed_list.append(current_vertex)

        # Gets the equivalent node from the graph
        current_vertex = getSameNode(current_vertex, graph_vertices)

        # Checks if the goal is within the radius specified
        # in the utils file
        if pointEncompassed(current_vertex, goal_vertex):
            print(' - - - GOAL FOUND - - - ')
            # Sets the value to True
            goal_reached = 1

        # A dictionary containing the neighbours and the weight/cost to reach them
        neighbours_dictionary = cost_graph_generated.getNeighbors(current_vertex).items()

        for neighbour, weight in neighbours_dictionary:

            # Gets the equivalent node from the graph
            neighbour = getSameNode(neighbour, graph_vertices)

            # Checking if the neighbour is in the closed list
            if checkinThis(neighbour, closed_list):
                continue

            # Adds the weight to the distance to reach the current node so far
            neighbour_g_cost = current_distance + weight
            # h_cost = 0, for Dijkstra,
            # Can have an Euclidean, or a Manhattan Heuristic
            neighbour_h_cost = EuclideanHeuristic(neighbour, goal_vertex)
            # f_cost is the sum of h_cost and g_cost
            neighbour_f_cost = neighbour_g_cost + neighbour_h_cost

            # If the distance to the node is less than the
            # previously stored distance to that neighbour,
            if neighbour_f_cost < distances[neighbour]:

                # draws the circle
                cv2.circle(map_canvas, (neighbour.x, neighbour.y), resolution, visited_colour, -1, cv2.LINE_AA)

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

                priority_queue.insert_pq(neighbour_f_cost, neighbour)


# Main function to run A Star
if __name__ == "__main__":
    final_img = map_canvas.copy()
    node_start = mouse_start_node
    node_goal = mouse_goal_node

    # Run the A Star Solve Function
    A_Star_Solve(cost_graph_generated, node_start, node_goal)
