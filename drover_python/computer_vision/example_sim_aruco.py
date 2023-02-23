#!/usr/bin/env python3

import time
from loguru import logger as log
import numpy as np
from fiducial_detector import FiducialDetector
from camera import SimCamera

if __name__ == "__main__":
    # idk what im doing but its kinda sorta closeish
    camera_matrix = np.array([[1080, 0,  1080/2],
                              [0,  720,  720/2],
                              [0,  0,    1]], dtype=np.float32)

    dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    cam = SimCamera()
    detector = FiducialDetector(camera_matrix, dist_coeffs, camera=cam)

    # record and detect with a marker length of 20 cm
    detector.loop_detect(0.2)
