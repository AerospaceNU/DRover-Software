#!/usr/bin/env python3

import time
import numpy as np
from loguru import logger as log
from drover import FiducialDetector, OpenCVCamera

if __name__ == "__main__":
    # random camera data I pulled from a year ago
    # this should be replaced with camera calibration values
    camera_matrix = np.array([[2600.2,  0,      1621.0],
                              [0,       2606.2, 1191.8],
                              [0,       0,      1]], dtype=np.float32)

    dist_coeffs = np.array([.1868, -0.3992, 0, 0, 0], dtype=np.float32)
    cam = OpenCVCamera(camera_matrix, dist_coeffs, width=3264, height=2448, fps=15, fourcc="MJPG")

    detector = FiducialDetector(camera=cam, display=True)

    while True:
        time.sleep(1)
        markers = detector.get_seen_markers()

        if markers is not None:
            log.info(f"Markers detected: {len(markers)}\n{markers}")
