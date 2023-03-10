
"""
Mission Controller for DRover
"""

import time
from dataclasses import dataclass
from typing import List
from .drone import Drone


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

    def circle_mission(self, radius=10.0, speed=1.0):
        """ Run a mission that circles all waypoints """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            # TODO yaw, clockwise, laps
            self._drone.circle_NEU(waypoint.x, waypoint.y, waypoint.alt, radius, speed=4, laps=2, yaw=90)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()
