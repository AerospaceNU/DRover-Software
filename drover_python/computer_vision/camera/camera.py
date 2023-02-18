#!/usr/bin/env python3

import abc


class Camera(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def get_frame(self):
        """ Grab a frame from the camera (waiting for the next frame) """
        pass
