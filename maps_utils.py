# Dimension of the square maze
map_size = 520
# Border thickness of the maze
border_size = 10
# Set Obstacles to be True, if the map has
# to be more accurate and for the path to not touch
# the borders
Obstacles = False
# Radius of the robot moving within the maze
resolution = 5
# Color of the path traced out by the robot within
# the maze - for visualization purposes
path_colour = (200, 25, 25)
# Mouse click color of the start - for visualization purposes
start_colour = (0, 0, 255)
# Mouse click color of the goal - for visualization purposes
goal_colour = (0, 255, 0)


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
    print('Left Click a point as a start Point')
    print('Right Click a point as a goal Point')
    print('Hit -Esc- after that')
