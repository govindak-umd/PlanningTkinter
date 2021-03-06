import cv2
from map import map_canvas, mouse_start_node, mouse_goal_node
from graph import graph_generated, checkinThis, printNode
from maps_utils import resolution, path_colour, pointEncompassed, visited_colour


# Breadth First Search Class
class BreadthFirstSearch:

    def __init__(self, graph, start_node, goal_node):
        self.graph = graph
        self.start_node = start_node
        self.goal_node = goal_node
        self.visited = []
        self.queue = []
        self.goal_reached = False
        self.count = 0

    def solveBreadthFirstSearch(self, node):
        self.visited.append(node)
        self.queue.append(node)

        while self.queue and self.goal_reached is False:

            queue_lifo = self.queue.pop(0)
            print('Now checking in the neighbour of ')
            print('-----')
            printNode(queue_lifo)
            print('-----')
            neighbours = self.graph.getNeighbors(queue_lifo)
            if neighbours is not None:
                for neighbour in neighbours:
                    if not checkinThis(neighbour, self.visited):
                        self.visited.append(neighbour)
                        self.queue.append(neighbour)
                        printNode(neighbour)

                        # Checking if the goal is within the radius
                        if pointEncompassed(neighbour, self.goal_node):
                            print('Length of Visited :', len(self.visited))
                            print('Length of Queue :', len(self.queue))
                            print(' - - - Goal Reached - - - ')
                            self.goal_reached = True
                        cv2.circle(map_canvas, (neighbour.x, neighbour.y), resolution, visited_colour, -1, cv2.LINE_AA)
                        cv2.imshow("Searching map", map_canvas)
                        if cv2.waitKey(20) & 0xFF == ord('q'):
                            break


# Main function to run the BFS
if __name__ == "__main__":
    node_start = mouse_start_node
    node_goal = mouse_goal_node
    bfs = BreadthFirstSearch(graph_generated, node_start, node_goal)
    bfs.solveBreadthFirstSearch(node_start)
