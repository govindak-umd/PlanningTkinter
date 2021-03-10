import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import getSameNode, cost_graph_generated, graph_generated, checkinThis, printNode
from maps_utils import resolution, path_colour, pointEncompassed, visited_colour
import heapq


def DijkstraSolve(graph, starting_vertex):
    graph_vertices = graph.getVertices()
    distances = {vertex: float('infinity') for vertex in graph_vertices}
    starting_vertex = getSameNode(starting_vertex, graph_vertices)
    distances[starting_vertex] = 0

    priority_queue = [(0, starting_vertex)]

    while len(priority_queue) > 0:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        current_vertex = getSameNode(current_vertex, graph_vertices)

        if current_distance > distances[current_vertex]:
            continue
        dict_C_items = cost_graph_generated.getNeighbors(current_vertex).items()

        for neighbour, weight in dict_C_items:
            distance = current_distance + weight
            neighbour = getSameNode(neighbour, graph_vertices)
            if distance < distances[neighbour]:
                distances[neighbour] = distance
                neighbour = getSameNode(neighbour, graph_vertices)
                heapq.heappush(priority_queue, (distance, neighbour))
    return distances


# Main function to run Dijkstra
if __name__ == "__main__":
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    DijkstraSolve(cost_graph_generated, node_start)
