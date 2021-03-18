# Dimension of the square maze
map_size = 250
# Border thickness of the maze
border_size = 10
# Set Obstacles to be True, if the map has
# to be more accurate and for the path to not touch
# the borders
Obstacles = True
# Radius of the robot moving within the maze
resolution = 8
# Color of the path traced out by the robot within
# the maze - for visualization purposes
path_colour = (200, 25, 25)
# Color of the visited nodes by the robot within
# the maze - for visualization purposes
visited_colour = (100, 120, 50)
# Mouse click color of the start - for visualization purposes
start_colour = (0, 0, 255)
# Mouse click color of the goal - for visualization purposes
goal_colour = (0, 255, 0)
# Cost to neighbours
cost = 1


# Node class
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y


# Check if a point is in a circle of
# resolution unit radius
def pointEncompassed(curr_node, goal_node):
    if ((curr_node.x - goal_node.x) ** 2 + (curr_node.y - goal_node.y) ** 2) < resolution ** 2:
        return True


# Start message to be printed with running instructions
def startMessage():
    print('Remember to set Obstacles to True, if you want a better')
    print('graph traversal, without hitting obstacles.')
    print('Left Click a point as a start Point')
    print('Right Click a point as a goal Point')
    print('Hit -Esc- after that')


def DistanceBetween(start_node, goal_node):
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


def checkInObstacle(node_check, map_img):
    """
    Check if a point is in the obstacle map.

    Returns true if it IS in the obstalce
    :param node_check: node to check
    :type node_check: Node
    :param map_img: Image of the map
    :type map_img: Numpy array of 3 by 3 dimension
    :return: True or False
    :rtype: Boolean
    """
    if map_img[node_check.x, node_check.y][0] == 0:
        return True
    return False
