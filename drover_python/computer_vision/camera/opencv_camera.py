#!/usr/bin/env python3

import cv2
from camera import Camera
import numpy as np


class OpenCVCamera(Camera):
    """ OpenCV's camera interface """

    def __init__(self, camera_id: int = 0, width: int = None, height: int = None, fps: int = None):
        """ Construct a new OpenCVCamera object """
        self.cap = cv2.VideoCapture(camera_id)

        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps is not None:
            self.cap.set(cv2.CAP_PROP_FPS, fps)

    def get_frame(self) -> np.ndarray:
        """ Get the latest image from the camera """
        return self.cap.read()[1]

    def __del__(self):
        """ Deconstructor """
        self.cap.release()
