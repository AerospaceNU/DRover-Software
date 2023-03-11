#!/usr/bin/env python3

import time
from loguru import logger as log
import numpy as np
from drover import FiducialDetector, SimCamera

if __name__ == "__main__":
    # idk what im doing but its kinda sorta closeish
    cam = SimCamera()
    detector = FiducialDetector(camera=cam, display=True)

    while True:
        time.sleep(1)
        markers = detector.get_seen_markers()

        if markers is not None:
            log.info(f"Markers detected: {len(markers)}\n{markers}")
