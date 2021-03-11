from map import map_canvas
from maps_utils import Node, resolution, map_size, border_size, Obstacles
from maps_utils import cost


def compareNodes(node_1, node_2):
    """
    Compares two nodes to check if they are equal
    
    :param      node_1:  The first node to check
    :type       node_1:  Node type
    :param      node_2:  The second node to check
    :type       node_2:  Node type
    
    :returns:   True or False
    :rtype:     Boolean type
    """
    if (node_1.x == node_2.x) and (node_1.y == node_2.y):
        return True
    else:
        return False


def checkinThis(node_to_check, set_to_check_in):
    """
    Checks if a node is there in a list or a set
    
    :param      node_to_check:    The node to check
    :type       node_to_check:    Node
    :param      set_to_check_in:  The set to check in
    :type       set_to_check_in:  List, Set
    
    :returns:   True or False
    :rtype:     Boolean type
    """
    for node in set_to_check_in:
        if compareNodes(node, node_to_check):
            return True
    return False


def getSameNode(node_to_check, set_to_check_in):
    """
    Gets the same node from the set or the list.
    
    :param      node_to_check:    The node to check for
    :type       node_to_check:    Node
    :param      set_to_check_in:  The set/list to check in
    :type       set_to_check_in:  Set or List
    
    :returns:   The equivalent node with the same x and y coordinate
    :rtype:     The node, else 0
    """
    for node in set_to_check_in:
        if compareNodes(node, node_to_check):
            return node
    return 0


def checkinGraph(node_to_check, graph_to_check_in):
    """
    Checks for teh nnode in the graph, by looking at all the 
    keys / parent nodes
    
    :param      node_to_check:      The node to check for
    :type       node_to_check:      Node type
    :param      graph_to_check_in:  The graph to check in
    :type       graph_to_check_in:  Graph type
    
    :returns:   True or False
    :rtype:     Boolean type
    """
    graph_keys = list(graph_to_check_in.getVertices())
    for node in graph_keys:
        if compareNodes(node, node_to_check):
            return True
    return False


def printNode(node):
    """
    Prints the node, making it easy for debugging
    
    :param      node:  The node
    :type       node:  Node type
    
    :returns: None
    :rtype:   None
    """
    print('Node is : ', node.x, ',', node.y)


class Graph:
    """
    This is a class that describes the entire map as a graph
    """

    def __init__(self, graph_dict):

        self.graph_dict = graph_dict

    def getVertices(self):
        """
        Gets the vertices of the graph
        
        :returns:   The vertices of the graph
        :rtype:     list
        """
        key_list = list(self.graph_dict.keys())
        return key_list

    def getNeighbors(self, node):
        """
        Gets the neighbors of every vertex
        
        :param      node:  The parent node
        :type       node:  Node
        
        :returns:   The neighbors / adjacent nodes to the parent
        :rtype:     Graph
        """
        key_list = list(self.graph_dict.keys())
        if checkinThis(node, key_list):
            similar_node = getSameNode(node, key_list)
            return self.graph_dict[similar_node]

    def getEdges(self):
        """
        Gets the edges of the graph
        
        :returns:   The edges of the graph
        :rtype:     set
        """
        val_set = set()
        val_node_list = list(self.graph_dict.values())
        for val_nodes in val_node_list:
            for val in val_nodes:
                val_set.add(val)
        return val_set


def generateCostGraph():

    print('Generating Cost Graph')
    cost_graph = {}
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
                    cost_graph[node] = {}

                    # Child Nodes

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range, y_range - resolution][0]
                    g = map_canvas[x_range, y_range - resolution][1]
                    r = map_canvas[x_range, y_range - resolution][2]
                    if b != 0 and g != 0 and r != 0:
                        node_top = Node(x_range, y_range - resolution)
                        cost_graph[node][node_top] = cost

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range, y_range + resolution][0]
                    g = map_canvas[x_range, y_range + resolution][1]
                    r = map_canvas[x_range, y_range + resolution][2]
                    if b != 0 and g != 0 and r != 0:
                        node_below = Node(x_range, y_range + resolution)
                        cost_graph[node][node_below] = cost

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range + resolution, y_range][0]
                    g = map_canvas[x_range + resolution, y_range][1]
                    r = map_canvas[x_range + resolution, y_range][2]
                    if b != 0 and g != 0 and r != 0:
                        node_right = Node(x_range + resolution, y_range)
                        cost_graph[node][node_right] = cost

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range - resolution, y_range][0]
                    g = map_canvas[x_range - resolution, y_range][1]
                    r = map_canvas[x_range - resolution, y_range][2]
                    if b != 0 and g != 0 and r != 0:
                        node_left = Node(x_range - resolution, y_range)
                        cost_graph[node][node_left] = cost

            # When obstacles are NOT present
            else:
                # Parent Node

                node = Node(x_range, y_range)
                cost_graph[node] = {}

                # Child Nodes
                node_left = Node(x_range - resolution, y_range)
                cost_graph[node][node_left] = cost

                node_right = Node(x_range + resolution, y_range)
                cost_graph[node][node_right] = cost

                node_below = Node(x_range, y_range + resolution)
                cost_graph[node][node_below] = cost

                node_top = Node(x_range, y_range - resolution)
                cost_graph[node][node_top] = cost
    # Assigning the graph with all the connections
    cost_graph_img = Graph(cost_graph)
    print('Cost Graphs updated')
    return cost_graph_img


def generateGraph():
    """
    Generates the graph, from the critical map dimensions
    
    :returns:   graph_img, a graph
    :rtype:     Graph
    """
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
                    b = map_canvas[x_range, y_range - resolution][0]
                    g = map_canvas[x_range, y_range - resolution][1]
                    r = map_canvas[x_range, y_range - resolution][2]
                    if b != 0 and g != 0 and r != 0:
                        node_top = Node(x_range, y_range - resolution)
                        graph_dic[node].append(node_top)

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range, y_range + resolution][0]
                    g = map_canvas[x_range, y_range + resolution][1]
                    r = map_canvas[x_range, y_range + resolution][2]
                    if b != 0 and g != 0 and r != 0:
                        node_below = Node(x_range, y_range + resolution)
                        graph_dic[node].append(node_below)

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range + resolution, y_range][0]
                    g = map_canvas[x_range + resolution, y_range][1]
                    r = map_canvas[x_range + resolution, y_range][2]
                    if b != 0 and g != 0 and r != 0:
                        node_right = Node(x_range + resolution, y_range)
                        graph_dic[node].append(node_right)

                    # Checking for the child node to not be in a
                    # boundary / obstacle
                    b = map_canvas[x_range - resolution, y_range][0]
                    g = map_canvas[x_range - resolution, y_range][1]
                    r = map_canvas[x_range - resolution, y_range][2]
                    if b != 0 and g != 0 and r != 0:
                        node_left = Node(x_range - resolution, y_range)
                        graph_dic[node].append(node_left)

            # When obstacles are NOT present
            else:
                # Parent Node

                node = Node(x_range, y_range)
                graph_dic[node] = []

                # Child Nodes
                node_left = Node(x_range - resolution, y_range)
                graph_dic[node].append(node_left)

                node_right = Node(x_range + resolution, y_range)
                graph_dic[node].append(node_right)

                node_below = Node(x_range, y_range + resolution)
                graph_dic[node].append(node_below)

                node_top = Node(x_range, y_range - resolution)
                graph_dic[node].append(node_top)
    # Assigning the graph with all the connections
    graph_img = Graph(graph_dic)
    print('Graphs updated')
    return graph_img


# Generating the graph
graph_generated = generateGraph()

# Generating a cost graph
cost_graph_generated = generateCostGraph()

# ----TESTING-----
# Generate Vertices of the graph
# vertices = graph_generated.getVertices()
# vertex_sample = vertices[0]
# printNode(vertex_sample)
# n = graph_generated.getNeighbors(vertex_sample)
# for i in n:
#     printNode(i)

# Gets the neighbours that shows the costs
# vertices = cost_graph_generated.getVertices()

# for vertex in vertices:
#     neighbour = cost_graph_generated.getNeighbors(vertex)
#     print(neighbour)
# ----TESTING COMPLETE-----