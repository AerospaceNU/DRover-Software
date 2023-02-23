#!/usr/bin/env python3

import cv2
import math
import numpy as np
from datetime import date
from camera.camera import Camera
from camera.opencv_camera import OpenCVCamera
from loguru import logger as log


class FiducialDetector():
    """ Detects ArUco fiducials in a video stream """

    @log.catch
    def __init__(self,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.ndarray,
                 camera: Camera = None,
                 dictionary: int = cv2.aruco.DICT_4X4_50):

        # dictionary should be denoted cv2.aruco.DICT_#X#_##
        if dist_coeffs.ndim != 1:
            raise ValueError("[Error] camera_matrix must be 2-dimensional")

        if dist_coeffs.ndim != 1:
            raise ValueError("[Error] dist_coeffs (distortion coefficients) "
                             "must be 1-dimensional")

        self.dictionary = dictionary
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        # get camera object
        if camera is None:
            self.camera = OpenCVCamera()
        else:
            self.camera = camera

    @staticmethod
    def euclidean_distance(tvecs):
        """ Calculate the euclidean distance between the camera
            and a fiducial via translation vectors

        Args:
            tvecs (2D NumPy Array): 3X1 NumPy Array

        Returns:
            float: Distance between camera and a fiducial
        """
        d = math.sqrt(float(tvecs[0][0])**2 +
                      float(tvecs[1][0])**2 +
                      float(tvecs[2][0])**2)
        return d

    def estimate_pose(self, marker_length: float, marker_corners: np.ndarray):
        """ Produces the transformational vectors required to get
            the camera to one aruco marker

        Args:
            marker_length (float): Length of the side of one
                                   of the aruco markers in meters
            markerCorners (np.ndarray): Array of the positions in frame of the
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
                                       self.camera_matrix,
                                       self.dist_coeffs)

        return (rvec, tvec)

    def draw_outline_and_data(self,
                              frame: np.ndarray,
                              markerCorners: np.ndarray,
                              markerID: int,
                              tvecs: np.ndarray):
        """ Draws the outline and location of a single fiducial
            over an image of the marker

        Args:
            frame (np.ndarray): The image within which the fiducial is
            markerCorners (np.ndarray): The four locations in the frame where
                                    the marker's corners are, ordered
                                    topleft, topright, bottomright, bottomleft
            markerID (int): The ID of the marker we are drawing on
            tvecs (np.ndarray): The vector from the camera to the marker
        """
        # DRAWING MARKERS, SOURCE:
        # https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

        # extract the marker corners (which are always returned
        # in top-left, top-right, bottom-right, and bottom-left
        # order)
        corners = markerCorners.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        # draw the bounding box of the ArUCo detection
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the
        # ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the frame
        cv2.putText(frame,
                    str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        # draw distance away from camera
        cv2.putText(frame,
                    f"[{tvecs[0, 0]:.2f}, {tvecs[1, 0]:.2f}, {tvecs[2, 0]:.2f}]",
                    (topLeft[0], topLeft[1] - 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

    def loop_detect(self, marker_length: float = 0.2, record: bool = False):
        """ Begins the camera and starts detection for markers

        Args:
            marker_length (float): The side length (in meters) of the markers
        """

        # load dictionary and detector parameters
        log.info("Loading 4X4_50 tags...")
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.dictionary)
        aruco_params = cv2.aruco.DetectorParameters()

        # initialize the video stream and allow the camera sensor to warm up
        log.info("Starting video stream...")

        # init a detector with dictionary and parameters
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # init mp4 writer
        if record:
            log.info("Recording video...")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            frame = self.camera.get_frame()
            video_writer = cv2.VideoWriter(f'aruco_record_{date.today():%m%d%Y}.mp4', fourcc, 30.0, (frame.shape[1], frame.shape[0]))

        while True:
            # grab a camera frame
            frame = self.camera.get_frame()

            # use that detector to check the frame
            corners, ids, rejected = detector.detectMarkers(frame)

            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for markerCorner, markerID in zip(corners, ids):
                    rvec, tvec = self.estimate_pose(marker_length, markerCorner)
                    # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    self.draw_outline_and_data(frame, markerCorner,
                                               markerID, tvec)

            # save frame
            if record:
                video_writer.write(frame)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        if record:
            video_writer.release()
