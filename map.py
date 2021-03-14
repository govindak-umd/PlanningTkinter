import cv2
import numpy as np
from maps_utils import map_size, start_colour, goal_colour, resolution, border_size, Node, startMessage

# Reads out the message
startMessage()
# Creates an empty image
map_canvas = np.zeros((map_size, map_size, 3), np.uint8)
# Dummy start node - Origin - First points of the image
mouse_start_node = Node(0, 0)
# Dummy goal node - Last point of the image
mouse_goal_node = Node(map_size, map_size)


def LoadMap(canvas, border):
    """
    Loads the map
    
    :param      canvas:  The blank canvas
    :type       canvas:  np matrix
    :param      border:  The border
    :type       border:  int
    """
    # Fill image
    for row in range(map_size):
        for col in range(map_size):
            canvas[row, col] = (255, 255, 255)

    # Top border
    for row in range(border):
        for col in range(map_size):
            canvas[row, col] = (0, 0, 0)

    # Left Border
    for row in range(map_size):
        for col in range(border):
            canvas[row, col] = (0, 0, 0)

    # Bottom Border
    for row in range(map_size - border, map_size):
        for col in range(map_size):
            canvas[row, col] = (0, 0, 0)

    # Right Border
    for row in range(map_size):
        for col in range(map_size - border, map_size):
            canvas[row, col] = (0, 0, 0)

    return canvas


def clickMouse(event, x, y, flags, params):
    """
    Mouse click event to register right and left clicks
    
    :param      event:   The MOUSECLICK event
    :type       event:   cv2 event
    :param      x:       x cooridnate of the clikc
    :type       x:       int
    :param      y:       y cooridnate of the click
    :type       y:       int
    :param      flags:   The flags
    :type       flags:   Boolean
    :param      params:  The parameters
    :type       params:  
    """
    global map_canvas
    global mouse_start_node
    global mouse_goal_node
    if event == cv2.EVENT_LBUTTONDOWN:
        print('Start at (', x, ',', y, ')')
        cv2.circle(map_canvas, (x, y), resolution, start_colour, -1, cv2.LINE_AA)
        cv2.imshow("map", map_canvas)
        mouse_start_node = Node(x, y)
    if event == cv2.EVENT_RBUTTONDOWN:
        print('Goal at (', x, ',', y, ')')
        map_canvas[x, y] = (0, 255, 0)
        cv2.circle(map_canvas, (x, y), resolution, goal_colour, -1, cv2.LINE_AA)
        cv2.imshow("map", map_canvas)
        mouse_goal_node = Node(x, y)


# Load the blank canvas
map_canvas = LoadMap(map_canvas, border_size)

# Record mouse clicks
while True:
    cv2.imshow("map", map_canvas)
    cv2.setMouseCallback('map', clickMouse)
    if cv2.waitKey(0) & 0xFF == 27:
        break
cv2.destroyAllWindows()
