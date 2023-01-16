# import the necessary packages
import time
import cv2
import numpy as np
import math


def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation=inter)

    # return the resized imagether
    return resized

def eucleidianDistance(tvecs):
    d = math.sqrt(float(tvecs[0][0])**2 + float(tvecs[1][0])**2 + float(tvecs[2][0])**2)
    return d





# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting 4X4_50 tags...")
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()
# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

time.sleep(2.0)
# loop over the frames from the video stream

cap = cv2.VideoCapture(0)


cameraMatrix = np.array([[ 9.1406711342366373e+02, 0., 6.6434795372667816e+02], [0.,
       9.1737281659181724e+02, 3.6009811226469412e+02], [0., 0., 1. ]])

distCoeffs = np.array([ 4.8465132033035055e-02, 6.0184689969411498e-01,
       -9.3990669110755216e-03, 5.0905051059619818e-03,
       -2.1865019660571408e+00 ])

while(True):
    ret, frame = cap.read()
    

    #ids = np.array([])
    # detect ArUco markers in the input frame

    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

    (corners, ids, rejected) = detector.detectMarkers(frame)

    # corners is Nx4, each row of corners is a marker's corners starting with top left and going counter clockwise 
    # len(corners) is equal to the number of detected markers

    drawBoxes = True

    #verify *at least* one ArUco marker was detected
    if len(corners) > 0 and drawBoxes:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        i = 0
        for (markerCorner, markerID) in zip(corners, ids):

            

            # 20 cm side lengths
            MARKER_LENGTH = 0.13

            objPoints = np.array([[-MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0], 
            [MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0], 
            [MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0], 
            [-MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0]])

            # np.float32 because of https://stackoverflow.com/questions/54249728/opencv-typeerror-expected-cvumat-for-argument-src-what-is-this

            rvecs, tvecs, _ = cv2.solvePnP(objPoints, markerCorner, np.float32(cameraMatrix), distCoeffs)
    

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
            # cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)
            i += 1



    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()




