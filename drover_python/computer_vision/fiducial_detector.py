#!/usr/bin/env python3

import cv2
import time
import numpy as np
from threading import Thread, Lock
from datetime import date
from camera import Camera, OpenCVCamera
from loguru import logger as log
from dataclasses import dataclass


@dataclass
class ArucoMarker:
    """ Dataclass representing an aruco marker """
    id: int
    location: np.ndarray = None
    rotation: np.ndarray = None
    last_seen: float = 0
    seen_counter: int = 0


class FiducialDetector():
    """ Asynchronously detects and tracks fiducials in a video stream """

    @log.catch
    def __init__(self,
                 camera: Camera = None,
                 dictionary: int = cv2.aruco.DICT_4X4_50,
                 size: float = 0.2,
                 display: bool = False,
                 frames_needed: int = 3,
                 marker_loss_timeout: float = 0.5):

        # get abstracted camera object to retrieve frames
        if camera is None:
            self.camera = OpenCVCamera()
        else:
            self.camera = camera

        self._dictionary = dictionary
        self._camera_matrix = self.camera.get_camera_matrix()
        self._dist_coeffs = self.camera.get_dist_coeffs()
        self._display = display
        self._frames_needed = frames_needed
        self._marker_loss_timeout = marker_loss_timeout
        self._marker_length = size
        self._aruco_dict = {}
        self._lock = Lock()

        # Run the camera thread
        self._camera_thread = Thread(target=self._run, daemon=True)
        self._camera_thread.start()

    def _estimate_pose(self, marker_length: float, marker_corners: np.ndarray):
        """ Produces the transformational vectors required to get
            the camera to one aruco marker

        Args:
            marker_length (float): Length of the side of one
                                   of the aruco markers in meters
            marker_corners (np.ndarray): Array of the positions in frame of the
                                         four corners of the marker in question

        Returns:
            Tuple: Tuple containing rotational vectors and translation
                   vectors to get to the marker
        """
        # the relative object coordinates of the marker in question
        obj_points = np.array([[-marker_length / 2, marker_length / 2, 0],
                               [marker_length / 2,  marker_length / 2, 0],
                               [marker_length / 2, -marker_length / 2, 0],
                               [-marker_length / 2, marker_length / 2, 0]])

        # np.float32 because of https://stackoverflow.com/a/55815108
        # documentation on this on OpenCV's website
        ret, rvec, tvec = cv2.solvePnP(obj_points,
                                       marker_corners,
                                       self._camera_matrix,
                                       self._dist_coeffs)

        return (rvec, tvec)

    def get_seen_markers(self):
        """ Returns a list of valid ArUco markers that are currently seen """
        markers = []

        for marker in self._aruco_dict.values():
            if marker.seen_counter >= self._frames_needed:
                markers.append(marker)

        return markers

    def _run(self):
        """ Runs the main loop of the program
        """
        # init dictionary and detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(self._dictionary)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        while True:
            frame = self.camera.get_frame()

            corners, ids, rejected = detector.detectMarkers(frame)

            # display markers
            if self._display:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                # cv2.drawFrameAxes(frame, self._camera_matrix, self._dist_coeffs, rvec, tvec, 0.1)
                cv2.imshow('aruco', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # continue if no markers found
            if len(corners) == 0:
                continue

            # update markers
            for c, id in zip(corners, ids):
                # convert to scalar
                assert (len(id) <= 1)
                id = id[0]

                rvec, tvec = self._estimate_pose(self._marker_length, c)

                if self._aruco_dict.get(id) is None:
                    self._aruco_dict[id] = ArucoMarker(id)

                # reset seen_counter if marker is lost
                if (time.time() - self._aruco_dict[id].last_seen) > self._marker_loss_timeout:
                    self._aruco_dict[id].seen_counter = 0

                self._aruco_dict[id].rotation = rvec
                self._aruco_dict[id].location = tvec
                self._aruco_dict[id].last_seen = time.time()
                self._aruco_dict[id].seen_counter += 1
