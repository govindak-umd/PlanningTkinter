import cv2
import os

# Degrees of freedom for the robot
# A 2 d.o.f robot can move in +x -x +y -y
# This can be developed further by getting a formula
# for finding a relation between the d.o.f and the directions,
# as well as between the d.o.f and the cost it takes for a path
dof_robot = 2


def GenerateVideo(image_folder, file_name, video_folder="Videos"):
    """
    Generate a Video from Images
    :param image_folder: Folder containing the Images
    :type image_folder: Directory
    :param file_name: Name of the file you wish to save
    :type file_name: string
    :param video_folder: Folder where the video should be saved
    :type video_folder: Directory
    """
    image_folder = image_folder
    video_name = video_folder + '/' + file_name + '.avi'

    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))

    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 5, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()
