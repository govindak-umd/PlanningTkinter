map_size = 520
border_size = 10
Obstacles = False


# Node class
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y


# Start message to be printed with running instructions

def startMessage():
    print('Left Click a point as a start Point')
    print('Right Click a point as a goal Point')
    print('Hit -Esc- after that')
