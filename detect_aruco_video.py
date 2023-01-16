

import time
import cv2
import numpy as np
import math

## IMAGE RESIZING INFO CAN BE FOUND HERE : https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

def euclideanDistance(tvecs):
    """ Calculate the euclidean distance between the camera and a fiducial via translation vectors

    Args:
        tvecs (2D NumPy Array): 3X1 NumPy Array

    Returns:
        float: Distance between camera and a fiducial
    """
    d = math.sqrt(float(tvecs[0][0])**2 + float(tvecs[1][0])**2 + float(tvecs[2][0])**2)
    return d

# load dictionary and detector parameters
print("[INFO] loading 4X4_50 tags...")
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

time.sleep(2.0)
# loop over the frames from the video stream

cap = cv2.VideoCapture(0)

# random camera data I pulled from a year ago, this should be replaced with camera calibration values
cameraMatrix = np.array([[ 9.1406711342366373e+02, 0., 6.6434795372667816e+02], [0.,
       9.1737281659181724e+02, 3.6009811226469412e+02], [0., 0., 1. ]])

distCoeffs = np.array([ 4.8465132033035055e-02, 6.0184689969411498e-01,
       -9.3990669110755216e-03, 5.0905051059619818e-03,
       -2.1865019660571408e+00 ])

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
        print(f'first: {type(ids)}')
        ids = ids.flatten()
        print(type(ids))
        # loop over the detected ArUCo corners
        i = 0
        for (markerCorner, markerID) in zip(corners, ids):


            ## POSE ESTIMATION ########################

            # marker side length in meters
            MARKER_LENGTH = 0.13

            # the relative object coordinates of the marker in question
            objPoints = np.array([[-MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0], 
            [MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0], 
            [MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0], 
            [-MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0]])

            # np.float32 because of https://stackoverflow.com/questions/54249728/opencv-typeerror-expected-cvumat-for-argument-src-what-is-this
            
            # documentation on this on OpenCV's website
            rvecs, tvecs, _ = cv2.solvePnP(objPoints, markerCorner, np.float32(cameraMatrix), distCoeffs)


            # DRAWING MARKERS, SOURCE : https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
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
            i += 1


    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()




