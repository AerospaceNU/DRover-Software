
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
    aruco_id: int = None
    aruco2_id: int = None

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
            self._drone.orbit_NEU(waypoint.x, waypoint.y, waypoint.alt, radius, speed=speed, laps=laps, yaw=yaw)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def spiral_mission(self, end_radius=30.0, start_radius=10, speed=2.0, laps=4):
        """ Run a mission that spirals all waypoints """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit_NEU(waypoint.x, waypoint.y, waypoint.alt, 
                                   start_radius, speed=speed, laps=laps, yaw=90,
                                   spiral_out_per_lap=(end_radius-start_radius)/laps)
            time.sleep(waypoint.wait_time)

        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def fiducial_search_mission(self, detector: FiducialDetector, 
                                start_radius=5, end_radius=20.0, speed=4.0, 
                                laps=4, max_dps=10):
        
        """ Run a mission that searches for aruco markers at waypoints """
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            # if no marker, just go to waypoint and continue
            if waypoint.aruco_id is None:
                log.info(f"Going to positional waypoint")
                self._drone.goto_NEU(waypoint.x, waypoint.y, waypoint.alt)
                time.sleep(waypoint.wait_time)
                continue

            # else if there is a marker, go to waypoint and search for marker
            if waypoint.aruco2_id is None:
                stop_func = lambda: (detector.get_seen(waypoint.aruco_id) is not None)
            else:
                stop_func = lambda: (detector.get_seen(waypoint.aruco_id) is not None or 
                                     detector.get_seen(waypoint.aruco2_id) is not None)
            not_found = self._drone.orbit_NEU(waypoint.x, waypoint.y, waypoint.alt, 
                                start_radius, speed=speed, laps=laps, yaw=0,
                                spiral_out_per_lap=(end_radius-start_radius)/laps,
                                stop_on_complete=False,
                                max_dps=max_dps, stop_function=stop_func)

            # wait for drone to come to a stand still
            #  TODO make not hard coded, 
            #  add go back and face marker last seen
            location = self._drone.get_location_NEU()
            yaw = self._drone.get_attitude()[2]
            self._drone.stop()
            self._drone.goto_NEU(location[0], location[1], location[2], yaw=yaw)
            time.sleep(4)

            # if marker(s) found, go there
            marker = detector.get_seen(waypoint.aruco_id)
            marker2 = detector.get_seen(waypoint.aruco2_id)
            if not_found:
                log.error("Marker(s) not found in search area")
            elif marker is not None and marker2 is not None:
                log.success(f"Found two markers ({waypoint.aruco_id}, {waypoint.aruco2_id})")
                self._drone.goto_NEU((marker.location[2]+marker2.location[2])/2, 
                                     (marker.location[0]+marker.location[0])/2, 
                                     0, relative=True)
            elif marker is not None:
                log.success(f"Found single marker ({waypoint.aruco_id})")
                self._drone.goto_NEU(marker.location[2], marker.location[0], 0, relative=True)
            elif marker2 is not None:
                log.success(f"Found single marker ({waypoint.aruco2_id})")
                self._drone.goto_NEU(marker2.location[2], marker2.location[0], 0, relative=True)
            else:
                log.error("Marker(s) lost after seen in search area")

            # wait there a bit
            time.sleep(waypoint.wait_time)

        # done with waypoints, go home and land
        self._drone.goto_NEU(0, 0, self._waypoints[0].alt)
        self._drone.land()
