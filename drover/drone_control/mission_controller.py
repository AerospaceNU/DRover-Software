
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

    def run_mission(self):
        """
        Begin the mission
        """

        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.goto_NEU(waypoint.x, waypoint.y, waypoint.alt)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()
