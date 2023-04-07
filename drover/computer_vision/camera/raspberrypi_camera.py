#!/usr/bin/env python3

import cv2
import numpy as np
from .camera import Camera
from picamera2 import Picamera2


class RaspberryPiCamera(Camera):
    """ Raspberrypi's camera interface """

    def __init__(self,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.array,
                 width: int = 1920,
                 height: int = 1080
                 ):
        """ Construct a new Raspberrypi object """
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'XRGB8888', 'size': (width, height)}))
        self.picam2.start()

    def get_frame(self) -> np.ndarray:
        """ Get the latest image from the camera """
        return self.picam2.capture_array()

    def get_camera_matrix(self):
        return self._camera_matrix

    def get_dist_coeffs(self):
        return self._dist_coeffs