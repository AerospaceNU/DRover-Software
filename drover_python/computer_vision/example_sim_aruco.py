#!/usr/bin/env python3

import time
from loguru import logger as log
import numpy as np
from fiducial_detector import FiducialDetector
from camera.sim_camera import SimCamera

if __name__ == "__main__":
    # random camera data I pulled from a year ago
    # this should be replaced with camera calibration values
    # camera_matrix = np.array([[914.07,  0,      664.35],
    #                           [0,       917.37, 360.10],
    #                           [0,       0,      1]], dtype=np.float32)

    # dist_coeffs = np.array([.04847, 0.60185, -.00940, .00509, -2.1865], dtype=np.float32)
    cam = SimCamera()
    # log.warning("Camera matrix: {}", cam.get_camera_mat())
    # detector = FiducialDetector(cam.get_camera_mat(), cam.get_dist_coeffs(), camera=cam)

    # record and detect with a marker length of 20 cm
    # detector.loop_detect(0.2)

    time.sleep(1000)
