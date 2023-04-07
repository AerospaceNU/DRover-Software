#!/usr/bin/env python3

import time
import numpy as np
from loguru import logger as log
from drover import FiducialDetector, RaspberryPiCamera

if __name__ == "__main__":
    # random camera data I pulled from a year ago
    # this should be replaced with camera calibration values
    camera_matrix = np.array([[2279.2,  0,      919.7696],
                              [0,       2289.9, 508.1499],
                              [0,       0,      1]], dtype=np.float32)

    dist_coeffs = np.array([-.4576, 0.3389, 0, 0, 0], dtype=np.float32)
    cam = RaspberryPiCamera(camera_matrix, dist_coeffs, width=1920, height=1080, fps=30)

    detector = FiducialDetector(camera=cam, display=True, display_scale=.5)

    while True:
        time.sleep(1)
        markers = detector.get_seen_markers()

        if markers is not None:
            log.info(f"Markers detected: {len(markers)}\n{markers}")
