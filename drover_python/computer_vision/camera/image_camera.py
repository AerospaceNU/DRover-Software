#!/usr/bin/env python3

import cv2
from camera.camera import Camera
import time


class ImageCamera(Camera):
    """ Camera that just displays a static image """

    def __init__(self, image_path: str = "images/aruco.jpg"):
        self.img = cv2.imread(image_path)

    def get_frame(self):
        """ Return copy of the image """
        time.sleep(0.01)  # 10fps
        return self.img.copy()
