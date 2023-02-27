#!/usr/bin/env python3

import time
import numpy as np
from loguru import logger as log
from fiducial_detector import FiducialDetector
from camera import OpenCVCamera

if __name__ == "__main__":
    # random camera data I pulled from a year ago
    # this should be replaced with camera calibration values
    camera_matrix = np.array([[914.07,  0,      664.35],
                              [0,       917.37, 360.10],
                              [0,       0,      1]], dtype=np.float32)

    dist_coeffs = np.array([.04847, 0.60185, -.00940, .00509, -2.1865], dtype=np.float32)
    cam = OpenCVCamera(camera_matrix, dist_coeffs, width=1920, height=1080, fps=30)

    detector = FiducialDetector(camera=cam, display=True)

    while True:
        time.sleep(1)
        markers = detector.get_seen_markers()

        if markers is not None:
            log.info(f"Markers detected: {len(markers)}\n{markers}")
