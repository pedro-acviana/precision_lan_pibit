import cv2
from picamera2 import Picamera2

class PicamCam:
    def __init__(self, resolution=(1280, 960)):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(main={"size": resolution})
        self.picam2.configure(config)
        self.picam2.start()
        self.resolution = resolution

    def get_next_image(self):
        frame = self.picam2.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)