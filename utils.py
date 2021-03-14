import cv2
import os


# def sortList(list)
def GenerateVideo(image_folder, file_name, video_folder="Videos"):
    image_folder = image_folder
    video_name = 'Videos/' + file_name + '.avi'

    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 5, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()
