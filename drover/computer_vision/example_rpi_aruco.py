#!/usr/bin/env python3

import time
import numpy as np
from loguru import logger as log
from drover import FiducialDetector, RaspberryPiCamera

if __name__ == "__main__":
    # random camera data I pulled from a year ago
    # this should be replaced with camera calibration values
    camera_matrix = np.array([[1499.09,  0,      968.87],
                              [0,       1498.23, 568.92],
                              [0,       0,      1]], dtype=np.float32)

    dist_coeffs = np.array([0.09383, 0.41789, 0, 0, 0], dtype=np.float32)
    cam = RaspberryPiCamera(camera_matrix, dist_coeffs, width=1920, height=1080)
    res = cam.get_frame().shape

    detector = FiducialDetector(camera=cam, display=True)

    while True:
        time.sleep(1)
        markers = detector.get_seen_markers()

        if markers is not None:
            log.info(f"Markers detected: {len(markers)}\n{markers}")
            log.debug(f"Resolution: {res[1]}x{res[0]}")
