#!/usr/bin/env python3

import abc


class Camera(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def get_frame(self):
        """ Grab a frame from the camera (waiting for the next frame) """
        pass

    @abc.abstractmethod
    def get_camera_matrix(self):
        """ Get the 3x3 camera matrix """
        return None

    @abc.abstractmethod
    def get_dist_coeffs(self):
        """ get the 1x6 set of distortion coefficients """
        return None
