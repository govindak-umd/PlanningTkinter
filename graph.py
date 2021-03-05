import cv2
from maps_utils import map_size
from maps_utils import border_size
from maps_utils import Obstacles
from map import map_canvas
from maps_utils import Node


# Function to compare nodes
def compareNodes(node_1, node_2):
    if (node_1.x == node_2.x) and (node_1.y == node_2.y):
        return True
    else:
        return False


# Checks if a node is there in a set
def checkinThis(node_to_check, set_to_check_in):
    for node in set_to_check_in:
        if compareNodes(node, node_to_check):
            return True
    return False


# Checks if a node is there in a set
def getSameNode(node_to_check, set_to_check_in):
    for node in set_to_check_in:
        if compareNodes(node, node_to_check):
            return node
    return 0


# Checks if a node is there in the graph
def checkinGraph(node_to_check, graph_to_check_in):
    graph_keys = list(graph_to_check_in.getVertices())
    for node in graph_keys:
        if compareNodes(node, node_to_check):
            return True
    return False


# Graph Class
def printNode(node):
    print('Node is : ', node.x, ',', node.y)


class Graph:
    def __init__(self, graph_dict):
        self.graph_dict = graph_dict

    # Returns all the vertices
    def getVertices(self):
        key_list = list(self.graph_dict.keys())
        return key_list

    # Returns the neighbours of the node
    def getNeighbors(self, node):
        key_list = list(self.graph_dict.keys())
        if checkinThis(node, key_list):
            similar_node = getSameNode(node, key_list)
            return self.graph_dict[similar_node]

    # Returns all the Edges
    def getEdges(self):
        val_set = set()
        val_node_list = list(self.graph_dict.values())
        for val_nodes in val_node_list:
            for val in val_nodes:
                val_set.add(val)
        return val_set


def generateGraph():
    print('Generating Graph')
    graph_dic = {}
    for x_range in range(border_size, map_size - border_size + 1):
        for y_range in range(border_size, map_size - border_size + 1):

            # When obstacles are present

            if Obstacles:
                # Getting all cell values to check for black cells
                b = map_canvas[x_range, y_range][0]
                g = map_canvas[x_range, y_range][1]
                r = map_canvas[x_range, y_range][2]

                # Adding only white cells into the graph
                if b != 0 and g != 0 and r != 0:
                    # Parent Node

                    node = Node(x_range, y_range)
                    graph_dic[node] = []

                    # Child Nodes

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range, y_range - 1][0]
                    g = map_canvas[x_range, y_range - 1][1]
                    r = map_canvas[x_range, y_range - 1][2]
                    if b != 0 and g != 0 and r != 0:
                        node_top = Node(x_range, y_range - 1)
                        graph_dic[node].append(node_top)

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range, y_range + 1][0]
                    g = map_canvas[x_range, y_range + 1][1]
                    r = map_canvas[x_range, y_range + 1][2]
                    if b != 0 and g != 0 and r != 0:
                        node_below = Node(x_range, y_range + 1)
                        graph_dic[node].append(node_below)

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range + 1, y_range][0]
                    g = map_canvas[x_range + 1, y_range][1]
                    r = map_canvas[x_range + 1, y_range][2]
                    if b != 0 and g != 0 and r != 0:
                        node_right = Node(x_range + 1, y_range)
                        graph_dic[node].append(node_right)

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range - 1, y_range][0]
                    g = map_canvas[x_range - 1, y_range][1]
                    r = map_canvas[x_range - 1, y_range][2]
                    if b != 0 and g != 0 and r != 0:
                        node_left = Node(x_range - 1, y_range)
                        graph_dic[node].append(node_left)

            # When obstacles are NOT present
            else:
                # Parent Node

                node = Node(x_range, y_range)
                graph_dic[node] = []

                # Child Nodes
                node_left = Node(x_range - 1, y_range)
                graph_dic[node].append(node_left)

                node_right = Node(x_range + 1, y_range)
                graph_dic[node].append(node_right)

                node_below = Node(x_range, y_range + 1)
                graph_dic[node].append(node_below)

                node_top = Node(x_range, y_range - 1)
                graph_dic[node].append(node_top)
    # Assigning the graph with all the connections
    graph_img = Graph(graph_dic)
    print('Graphs updated')
    return graph_img


# Generating the graph
graph_generated = generateGraph()
