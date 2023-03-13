
"""
Mission Controller for DRover
"""

import time
import random
import numpy as np
from loguru import logger as log
from dataclasses import dataclass
from typing import List
from drover import Drone, FiducialDetector, ArucoMarker

@dataclass
class Waypoint():
    """
    Mission waypoint
    """
    x: float
    y: float
    alt: float
    wait_time: float = 1.0
    use_latlon: bool = False
    aruco_id: int = -1
    aruco2_id: int = -1

    def move_random(self, dist):
        """ Moves waypoint `dist` in random direction """
        angle = random.uniform(0, 2*np.pi)
        x = dist*np.cos(angle)
        y = dist*np.sin(angle)
        self.x += x
        self.y += y


class MissionController():
    def __init__(self, drone: Drone, waypoints: List[Waypoint]):
        self._waypoints = waypoints.copy()
        self._drone = drone

    def simple_mission(self):
        """
        Run a mission that simply goes to each waypoint in order
        """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.goto_NEU(waypoint.x, waypoint.y, waypoint.alt)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def circle_mission(self, radius=10.0, speed=2.0, laps=1, yaw=90):
        """ Run a mission that circles all waypoints """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.circle_NEU(waypoint.x, waypoint.y, waypoint.alt, radius, speed=speed, laps=laps, yaw=yaw)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def spiral_mission(self, end_radius=30.0, start_radius=10, speed=2.0, laps=4):
        """ Run a mission that spirals all waypoints """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.circle_NEU(waypoint.x, waypoint.y, waypoint.alt, 
                                   start_radius, speed=speed, laps=laps, yaw=90,
                                   spiral_out_per_lap=(end_radius-start_radius)/laps)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def fiducial_search_mission(self, detector: FiducialDetector, start_radius=5, end_radius=20.0, speed=2.0, laps=4):
        """ Run a mission that spirals all waypoints """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            # go to waypoint and find marker
            stop_func = lambda: (detector.get(waypoint.aruco_id) is not None)
            not_found = self._drone.circle_NEU(waypoint.x, waypoint.y, waypoint.alt, 
                                   start_radius, speed=speed, laps=laps, yaw=90,
                                   spiral_out_per_lap=(end_radius-start_radius)/laps,
                                   stop_function=stop_func)

            # wait for drone to come to a stand still
            #  TODO make not hard coded
            time.sleep(4)

            marker = detector.get(waypoint.aruco_id)
            if not not_found and marker is not None:
                log.success(f"Found marker {waypoint.aruco_id}")
                self._drone.goto_NEU(marker.location[2], marker.location[0], 
                                     0, relative=True)

            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()
