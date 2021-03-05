import cv2
from maps_utils import map_size
from maps_utils import border_size
import numpy as np

map_canvas = np.zeros((map_size, map_size, 3), np.uint8)


def LoadMap(canvas, border):
    # Fill image
    for row in range(520):
        for col in range(520):
            canvas[row, col] = (255, 255, 255)

    # Top border
    for row in range(border):
        for col in range(520):
            canvas[row, col] = (0, 0, 0)

    # Left Border
    for row in range(520):
        for col in range(border):
            canvas[row, col] = (0, 0, 0)

    # Bottom Border
    for row in range(520 - border, 520):
        for col in range(520):
            canvas[row, col] = (0, 0, 0)

    # Right Border
    for row in range(520):
        for col in range(520 - border, 520):
            canvas[row, col] = (0, 0, 0)

    return canvas


def clickMouse(event, x, y, flags, params):
    global map_canvas
    if event == cv2.EVENT_LBUTTONDOWN:
        print('Start at (', x, ',', y, ')')
        cv2.circle(map_canvas, (x, y), 1, (0, 0, 255), -1, cv2.LINE_AA)
        # cv2.circle(map_canvas, (x, y), 10, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.imshow("map", map_canvas)
    if event == cv2.EVENT_RBUTTONDOWN:
        print('Goal at (', x, ',', y, ')')
        map_canvas[x, y] = (0, 255, 0)
        cv2.circle(map_canvas, (x, y), 1, (0, 255, 0), -1, cv2.LINE_AA)
        # cv2.circle(map_canvas, (x, y), 10, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow("map", map_canvas)


# Load the blank canvas
map_canvas = LoadMap(map_canvas, border_size)

# Record mouse clicks
while True:
    cv2.imshow("map", map_canvas)
    cv2.setMouseCallback('map', clickMouse)
    if cv2.waitKey(0) & 0xFF == 27:
        break
cv2.destroyAllWindows()
