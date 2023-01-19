#!/usr/bin/env python3
import numpy as np
import cv2

from fiducialdetector import FiducialDetector

# random camera data I pulled from a year ago, this should be replaced with camera calibration values
cameraMatrix = np.array([[ 9.1406711342366373e+02, 0., 6.6434795372667816e+02], [0.,
       9.1737281659181724e+02, 3.6009811226469412e+02], [0., 0., 1. ]])

distCoeffs = np.array([ 4.8465132033035055e-02, 6.0184689969411498e-01,
       -9.3990669110755216e-03, 5.0905051059619818e-03,
       -2.1865019660571408e+00 ])

detector = FiducialDetector(cameraMatrix, distCoeffs, cv2.aruco.DICT_4X4_50)

# record and detect with a marker length of 20 cm
detector.record_and_detect(0.2)