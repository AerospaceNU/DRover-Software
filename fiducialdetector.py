
import time
import cv2
import math
import numpy as np

class FiducialDetector():

    def __init__(self, cameraMatrix: np.ndarray, distCoeffs: np.ndarray, dictionary: int):
        ## dictionary should be denoted cv2.aruco.DICT_#X#_##
        if distCoeffs.ndim != 1:
            raise ValueError("[Error] cameraMatrix must be 2-dimensional")

        if distCoeffs.ndim != 1:
            raise ValueError("[Error] distCoeffs (distortion coefficients) must be 1-dimensional")

        self.dictionary = dictionary
        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs


    @staticmethod
    def euclideanDistance(tvecs):
        """ Calculate the euclidean distance between the camera and a fiducial via translation vectors

        Args:
            tvecs (2D NumPy Array): 3X1 NumPy Array

        Returns:
            float: Distance between camera and a fiducial
        """
        d = math.sqrt(float(tvecs[0][0])**2 + float(tvecs[1][0])**2 + float(tvecs[2][0])**2)
        return d


    def estimatePose(self, marker_length: float, markerCorners: np.ndarray):
        """ Produces the transformational vectors required to get the camera to one aruco marker

        Args:
            marker_length (float): Length of the side of one of the aruco markers in meters
            markerCorners (np.ndarray): Array of the positions in frame of the four corners of the marker in question

        Returns:
            Tuple: Tuple containing rotational vectors and translation vectors to get to the marker
        """
        # the relative object coordinates of the marker in question
        objPoints = np.array([[-marker_length / 2, marker_length / 2, 0], 
        [marker_length / 2, marker_length / 2, 0], 
        [marker_length / 2, -marker_length / 2, 0], 
        [-marker_length / 2, marker_length / 2, 0]])

        # np.float32 because of https://stackoverflow.com/questions/54249728/opencv-typeerror-expected-cvumat-for-argument-src-what-is-this
            
        # documentation on this on OpenCV's website
        rvecs, tvecs, _ = cv2.solvePnP(objPoints, markerCorners, np.float32(self.cameraMatrix), self.distCoeffs)

        return (rvecs, tvecs)


    def draw_outline_and_data(self, frame: np.ndarray, markerCorners: np.ndarray, markerID: int, tvecs: np.ndarray):
        """ draws the outline and location of a single fiducial over an image of the marker

        Args:
            frame (np.ndarray): The image within which the fiducial is
            markerCorners (np.ndarray): The four locations in the frame where the marker's corners are, ordered topleft, topright, bottomright, bottomleft
            markerID (int): The ID of the marker we are drawing on
            tvecs (np.ndarray): The translational vector from the camera to the marker
        """
        # DRAWING MARKERS, SOURCE : https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

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
        cv2.putText(frame, str(markerID),
            (topLeft[0], topLeft[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

        cv2.putText(frame, "[x, y, z] " + str(tvecs[2]),
            (topLeft[0], topLeft[1] - 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)



    def record_and_detect(self, marker_length: float):
        """ Begins the camera recording and starts detection for markers

        Args:
            marker_length (float): The side length (in meters) of the markers to be detected
        """
        # load dictionary and detector parameters
        print("[INFO] loading 4X4_50 tags...")
        arucoDict = cv2.aruco.getPredefinedDictionary(self.dictionary)
        arucoParams = cv2.aruco.DetectorParameters()
        # initialize the video stream and allow the camera sensor to warm up
        print("[INFO] starting video stream...")

        time.sleep(2.0)
        # loop over the frames from the video stream

        cap = cv2.VideoCapture(0)

        while(True):

            ret, frame = cap.read()
            #ids = np.array([])
            # detect ArUco markers in the input frame

            # initialize a detector of the dictionary and parameters we've initialized
            detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

            # use that detector to check the frame
            (corners, ids, rejected) = detector.detectMarkers(frame)

            drawBoxes = True

            #verify *at least* one ArUco marker was detected
            if len(corners) > 0 and drawBoxes:
            # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                i = 0
                for (markerCorner, markerID) in zip(corners, ids):
                    vecs = self.estimatePose(marker_length, markerCorner)
                    tvecs = vecs[1]
                    self.draw_outline_and_data(frame, markerCorner, markerID, tvecs)
                    i += 1

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()