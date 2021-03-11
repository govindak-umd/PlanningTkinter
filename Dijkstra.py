import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, cost_graph_generated, compareNodes, graph_generated, checkinThis, printNode
from maps_utils import resolution, pointEncompassed, visited_colour


class PriorityQueue:
    def __init__(self):
        self.queue = []

    def insert_pq(self, cost, data_node):
        print('Length of queue : >> ', len(self.queue))
        node_cost_combo = (cost, data_node)
        self.queue.append(node_cost_combo)

    def pop_pq(self):
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
        return len(self.queue)


def DijkstraSolve(graph, starting_vertex, goal_vertex):
    graph_vertices = graph.getVertices()
    distances = {vertex: float('infinity') for vertex in graph_vertices}
    starting_vertex = getSameNode(starting_vertex, graph_vertices)
    distances[starting_vertex] = 0
    goal_reached = False

    priority_queue = PriorityQueue()
    priority_queue.insert_pq(0, starting_vertex)

    while priority_queue.len_pq() > 0:

        current_distance, current_vertex = priority_queue.pop_pq()

        current_vertex = getSameNode(current_vertex, graph_vertices)

        if current_distance > distances[current_vertex]:
            continue

        dict_C_items = cost_graph_generated.getNeighbors(current_vertex).items()

        for neighbour, weight in dict_C_items:
            cv2.circle(map_canvas, (neighbour.x, neighbour.y), resolution, visited_colour, -1, cv2.LINE_AA)
            distance = current_distance + weight
            neighbour = getSameNode(neighbour, graph_vertices)
            if distance < distances[neighbour]:
                distances[neighbour] = distance
                printNode(neighbour)
                cv2.imshow("Searching map", map_canvas)
                if pointEncompassed(neighbour, goal_vertex):
                    print('GOAL FOUND')
                    goal_reached = True
                if cv2.waitKey(20) & 0xFF == ord('q'):
                    break
                neighbour = getSameNode(neighbour, graph_vertices)
                priority_queue.insert_pq(distance, neighbour)
    return distances


# Main function to run Dijkstra
if __name__ == "__main__":
    pq_custom = PriorityQueue()
    node_start = mouse_start_node
    node_goal = mouse_goal_node

    # Run the Dijkstra Solve Function
    DijkstraSolve(cost_graph_generated, node_start, node_goal)

    # ----TESTING-----

    # print('Adding node to pq')
    # pq_custom.insert_pq(7, node_start)
    # print('Adding node to pq')
    # pq_custom.insert_pq(90, node_start)
    # pq_custom.insert_pq(99, node_start)
    # pq_custom.insert_pq(911, node_start)
    # pq_custom.insert_pq(911, node_goal)
    # returned_max_cost, returned_max_cost_node = pq_custom.pop_pq()
    # print('Returning Maximum cost node')
    # printNode(returned_max_cost_node)
    # print(returned_max_cost)
    # len_pq_custom =  pq_custom.len_pq()
    # print(len_pq_custom)

    # ----TESTING COMPLETE-----
