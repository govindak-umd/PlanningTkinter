import cv2
from maps_utils import map_size
from maps_utils import start_colour
from maps_utils import goal_colour
from maps_utils import border_size
import numpy as np
from maps_utils import Node
from maps_utils import startMessage
from maps_utils import resolution

startMessage()
map_canvas = np.zeros((map_size, map_size, 3), np.uint8)
mouse_start_node = Node(0, 0)
mouse_goal_node = Node(map_size, map_size)


def LoadMap(canvas, border):
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
    global map_canvas
    global mouse_start_node
    global mouse_goal_node
    if event == cv2.EVENT_LBUTTONDOWN:
        print('Start at (', x, ',', y, ')')
        cv2.circle(map_canvas, (x, y), resolution, start_colour, -1, cv2.LINE_AA)
        cv2.imshow("map", map_canvas)
        mouse_start_node = Node(y, x)
    if event == cv2.EVENT_RBUTTONDOWN:
        print('Goal at (', x, ',', y, ')')
        map_canvas[x, y] = (0, 255, 0)
        cv2.circle(map_canvas, (x, y), resolution, goal_colour, -1, cv2.LINE_AA)
        cv2.imshow("map", map_canvas)
        mouse_goal_node = Node(y, x)


# Load the blank canvas
map_canvas = LoadMap(map_canvas, border_size)

# Record mouse clicks
while True:
    cv2.imshow("map", map_canvas)
    cv2.setMouseCallback('map', clickMouse)
    if cv2.waitKey(0) & 0xFF == 27:
        break
cv2.destroyAllWindows()
