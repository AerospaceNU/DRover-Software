#!/usr/bin/env python3

import cv2
import time
import socket
import struct
import numpy as np
from loguru import logger as log
from threading import Thread, Lock
from .camera import Camera


class SimCamera(Camera):
    """ Webots camera interface """
    img_header_format = "=HH"
    img_header_size = struct.calcsize(img_header_format)

    def __init__(self, port=5599, width=640, height=480, fovx=45):
        """ Construct a new SimCamera object """

        self._latest_image = None
        self._focal_length = None
        self._img_lock = Lock()
        self._width = width
        self._height = height
        self._fovx = np.deg2rad(fovx)

        # connect to WebotsArduVehicle
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(("127.0.0.1", port))

        # wait for first image
        log.info("Waiting for first image from webots...")
        self._recv_image()
        log.info("SimCamera connected to Webots")

        Thread(target=self._run, daemon=True).start()

    def _recv_image(self):
        # receive header
        header = self.socket.recv(self.img_header_size)
        if len(header) != self.img_header_size:
            log.error("SimCamera header size mismatch")
            return None

        # parse header
        width, height = struct.unpack(self.img_header_format, header)

        # receive image
        bytes_to_read = width * height
        img = bytes()
        while len(img) < bytes_to_read:
            img += self.socket.recv(min(bytes_to_read - len(img), 2**20))

        # convert incoming bytes to a numpy array (a grayscale image)
        img = np.frombuffer(img, np.uint8).reshape((height, width))

        return img

    def _run(self):
        """ Receive images from WebotsArduVehicle """
        while True:
            # receive image
            img = self._recv_image()
            if img is None:
                break

            # update latest image safely
            self._img_lock.acquire()
            self._latest_image = img
            self._img_lock.release()

        self.socket.close()

    def get_frame(self) -> np.ndarray:
        """ Get the latest image from the camera """

        # wait for a new image
        while self._latest_image is None:
            time.sleep(0.001)

        # acquire lock so we can safely get the latest image
        self._img_lock.acquire()
        img = self._latest_image
        self._latest_image = None
        self._img_lock.release()

        # convert to bgr
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        return img

    def get_camera_matrix(self):
        """ Get the 3x3 camera matrix """
        # https://stackoverflow.com/questions/61555182/webot-camera-default-parameters-like-pixel-size-and-focus
        # Not quite correct but close
        f = self._width/np.tan(self._fovx/2)/2 
        return np.array([[f, 0, self._width/2],
                         [0, f, self._height/2],
                         [0, 0, 1]], dtype=np.float32)

    def get_dist_coeffs(self):
        """ Get 1x6 distortion array """
        return np.array([0, 0, 0, 0, 0], dtype=np.float32)

    def __del__(self):
        """ Deconstructor """
        self.socket.close()
