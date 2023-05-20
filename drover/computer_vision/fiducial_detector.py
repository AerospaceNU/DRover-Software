#!/usr/bin/env python3

import cv2
import time
import numpy as np
from threading import Thread, Lock
from loguru import logger as log
from dataclasses import dataclass
from drover import Camera, OpenCVCamera


@dataclass
class ArucoMarker:
    """ Dataclass representing an aruco marker """
    id: int
    image_location: int = np.zeros((2,1))
    location: np.ndarray = np.zeros((3,1))
    rotation: np.ndarray = np.zeros((3,1))
    last_seen: float = 0
    seen_counter: int = 0


class FiducialDetector():
    """ Asynchronously detects and tracks fiducials in a video stream """

    def __init__(self,
                 camera: Camera = None,
                 dictionary: int = cv2.aruco.DICT_4X4_50,
                 size: float = 0.15,
                 fullscreen:bool = False,
                 display: bool = False,
                 frames_needed: int = 3,
                 marker_loss_timeout: float = 0.5,
                 display_resize: float = (720, 540), # 4k to 480i ish
                 max_callback_rate: float = 10):

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
        self._marker_callbacks = []
        self._max_callback_rate = max_callback_rate
        self._last_callbacks = 0
        self.display_resize = display_resize
        self._fullscreen = fullscreen

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
                               [-marker_length / 2,-marker_length / 2, 0]])

        # np.float32 because of https://stackoverflow.com/a/55815108
        # documentation on this on OpenCV's website
        ret, rvec, tvec = cv2.solvePnP(obj_points,
                                       marker_corners,
                                       self._camera_matrix,
                                       self._dist_coeffs)

        return (rvec, tvec)

    def get_seen(self, id):
        """ Returns an ArucoMarker if seen, otherwise None """
        marker = self._aruco_dict.get(id)
        if (marker is not None and
            marker.seen_counter >= self._frames_needed and
            (time.time() - marker.last_seen) < self._marker_loss_timeout):
            return marker

        return None

    def get_latest(self, id):
        """ Returns the last ArucoMarker seen, if never seen returns None """
        return self._aruco_dict.get(id)

    def get_seen_markers(self):
        """ Returns a list of valid ArUco markers that are currently seen """
        markers = []
        for marker in self._aruco_dict.values():
            if (marker.seen_counter >= self._frames_needed and
               (time.time() - marker.last_seen) < self._marker_loss_timeout):
                markers.append(marker)

        return markers

    def register_marker_callback(self, handler_func):
        """ Registers a callback called when a maker is seen.
            Function is given a list of seen markers """
        self._marker_callbacks.append(handler_func)

    def _run(self):
        """ Runs the main loop of the program
        """
        # init dictionary and detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(self._dictionary)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        while True:
            frame = self.camera.get_frame()
            if frame is None or frame.size == 0:
                continue
            #log.debug(f"{frame.shape}")
            corners, ids, rejected = detector.detectMarkers(frame)

            # continue if no markers found
            if len(corners) == 0:
                if self._display:
                    resized = cv2.resize(frame, self.display_resize, interpolation = cv2.INTER_AREA)
                    cv2.imshow('aruco', resized)
                    cv2.waitKey(1)
                continue

            # update markers
            for c, id in zip(corners, ids):
                # convert to scalar
                assert (len(id) <= 1)
                id = id[0]

                rvec, tvec = self._estimate_pose(self._marker_length, c)

                # draw on frame
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self._camera_matrix, self._dist_coeffs, rvec, tvec, 0.1)

                if self._aruco_dict.get(id) is None:
                    self._aruco_dict[id] = ArucoMarker(id)

                # reset seen_counter if marker is lost
                if (time.time() - self._aruco_dict[id].last_seen) > self._marker_loss_timeout:
                    self._aruco_dict[id].seen_counter = 0

                self._aruco_dict[id].image_location = np.average(c[0], axis=0) / np.array([frame.shape[1], frame.shape[0]])
                self._aruco_dict[id].rotation = rvec.T[0]
                self._aruco_dict[id].location = tvec.T[0]
                self._aruco_dict[id].last_seen = time.time()
                self._aruco_dict[id].seen_counter += 1

            # display markers
            if self._display:
                resized = cv2.resize(frame, self.display_resize, interpolation = cv2.INTER_AREA)
                if self._fullscreen:
                    cv2.namedWindow("aruco", cv2.WINDOW_NORMAL)
                    cv2.setWindowProperty("aruco", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                cv2.imshow("aruco", resized)
                cv2.imshow('aruco', resized)
                cv2.waitKey(1)

            # run callbacks
            if (self._marker_callbacks and 
                (time.time()-self._last_callbacks) > 1/self._max_callback_rate):
                self._last_callbacks = time.time()
                markers = self.get_seen_markers()
                if len(markers) > 0:
                    for handler in self._marker_callbacks:
                        handler(markers)
