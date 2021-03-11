import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, cost_graph_generated, compareNodes, graph_generated, checkinThis, printNode
from maps_utils import resolution, pointEncompassed, visited_colour, path_colour


# Priority Queue Class
class PriorityQueue:
    """
    This class is Priority Queue.
    Look into python heapq for a similar usecase
    """

    def __init__(self):
        self.queue = []

    def insert_pq(self, cost, data_node):
        """
        Insert into the Priority queue
        
        :param      cost:       The cost
        :type       cost:       { type_description }
        :param      data_node:  The data node
        :type       data_node:  { type_description }
        """
        node_cost_combo = (cost, data_node)
        self.queue.append(node_cost_combo)

    def pop_pq(self):
        """
        Pops the element as per the rules of priority queue
        """
        try:
            max_idx = 0
            for i in range(len(self.queue)):
                if self.queue[i][0] > self.queue[max_idx][0]:
                    max_idx = i
            max_cost, max_cost_node = self.queue[max_idx]
            del self.queue[max_idx]
            return max_cost, max_cost_node
        except IndexError:
            exit()

    def len_pq(self):
        """
        Returns the length of the priority queue
        """
        print('Length of queue : >> ', len(self.queue))
        return len(self.queue)


def DijkstraSolve(graph, starting_vertex, goal_vertex):
    """
    Function to solve the graph through Dijkstra's Algorithm
    
    :param      graph:            The graph
    :type       graph:            Graph type
    :param      starting_vertex:  The starting vertex
    :type       starting_vertex:  Node type
    :param      goal_vertex:      The goal vertex
    :type       goal_vertex:      Node type
    """
    goal_reached = 0

    # Returns the vertices of the graph
    graph_vertices = graph.getVertices()
    distances = {vertex: float('infinity') for vertex in graph_vertices}
    distances[starting_vertex] = 0

    starting_vertex = getSameNode(starting_vertex, graph_vertices)

    # Initializes the Priority Queue
    priority_queue = PriorityQueue()
    priority_queue.insert_pq(0, starting_vertex)

    # Runs while loop until goal node is not found
    while priority_queue.len_pq() > 0 and goal_reached == 0:

        # Gets the node as per the priority queue
        current_distance, current_vertex = priority_queue.pop_pq()

        # Gets the equivalent node from the graph
        current_vertex = getSameNode(current_vertex, graph_vertices)

        if current_distance > distances[current_vertex]:
            continue

        # A dictionary containing the neighbours and the weight/cost to reach them
        neighbours_dictionary = cost_graph_generated.getNeighbors(current_vertex).items()

        for neighbour, weight in neighbours_dictionary:

            cv2.circle(map_canvas, (neighbour.x, neighbour.y), resolution, visited_colour, -1, cv2.LINE_AA)
            distance = current_distance + weight

            # Gets the equivalent node from the graph
            neighbour = getSameNode(neighbour, graph_vertices)

            # If the distance to the node is less than the 
            # previously stored distance to that neighbour,

            if distance < distances[neighbour]:

                # Replace the distance value

                distances[neighbour] = distance

                # Shows the traversal on map
                cv2.imshow("Searching map", map_canvas)

                # Checks if the goal is within the radius specified
                # in the utils file
                if pointEncompassed(neighbour, goal_vertex):
                    print(' - - - GOAL FOUND - - - ')
                    # Sets the value to True
                    goal_reached = 1

                if cv2.waitKey(20) & 0xFF == ord('q'):
                    break

                # Gets the equivalent node from the graph
                neighbour = getSameNode(neighbour, graph_vertices)

                # Inserts the new node back into the priority queue as 
                # per the rules of the Priority Queue Class.
                # This new neighbour will be the one that can be traversed
                # to with the lowest cost
                priority_queue.insert_pq(distance, neighbour)


# Main function to run Dijkstra
if __name__ == "__main__":
    final_img = map_canvas.copy()
    pq_custom = PriorityQueue()
    node_start = mouse_start_node
    node_goal = mouse_goal_node

    # Run the Dijkstra Solve Function
    DijkstraSolve(cost_graph_generated, node_start, node_goal)
