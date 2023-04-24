
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
    land: bool = False
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
    def __init__(self, drone: Drone, 
                 waypoints: List[Waypoint],
                 use_home_rally_point: bool = True):
        """Class handling drone missions

        Args:
            drone (Drone): Drone object
            waypoints (List[Waypoint]): List of waypoints in the mission
            use_home_rally_point (bool, optional): Sets a rally point before taking off. Defaults to True.
        """
        self._waypoints = waypoints.copy()
        self._drone = drone
        self._use_home_rally_point = use_home_rally_point

    def _pre_mission(self):
        """ Called at the beginning of every mission """
        if self._use_home_rally_point:
            self._drone.set_home_rally_point()
        
        # Upload waypoints as a mission for GCS visualization purposes
        mission_list = []
        for wp in self._waypoints:
            if wp.use_latlon:
                mission_list.append((wp.x, wp.y, wp.alt))
            else:
                lat, lon = self._drone.NEU_to_latlon(wp.x, wp.y)
                mission_list.append((lat, lon, wp.alt))
                
        self._drone.upload_mission_latlon(mission_list)


    def simple_mission(self):
        """
        Run a mission that simply goes to each waypoint in order
        """
        self._pre_mission()
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.goto(waypoint.x, waypoint.y, waypoint.alt)
            time.sleep(waypoint.wait_time)

        self._drone.goto(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def circle_mission(self, radius=10.0, speed=2.0, laps=1, yaw=90):
        """ Run a mission that circles all waypoints """
        self._pre_mission()
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit_NEU(waypoint.x, waypoint.y, waypoint.alt, radius, speed=speed, laps=laps, yaw=yaw)
            time.sleep(waypoint.wait_time)

        self._drone.goto(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def spiral_mission(self, end_radius=30.0, start_radius=10, speed=2.0, laps=4):
        """ Run a mission that spirals all waypoints """
        self._pre_mission()
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit_NEU(waypoint.x, waypoint.y, waypoint.alt, 
                                   start_radius, speed=speed, laps=laps, yaw=90,
                                   spiral_out_per_lap=(end_radius-start_radius)/laps)
            time.sleep(waypoint.wait_time)

        self._drone.goto(0, 0, self._waypoints[0].alt)
        self._drone.land()

    def fiducial_search_mission(self, detector: FiducialDetector, 
                                start_radius=5, end_radius=20.0, speed=4.0, 
                                laps=4, max_dps=10):
        """ Run a mission that searches for aruco markers at waypoints """
        self._pre_mission()
        ret = self._drone.arm_takeoff(self._waypoints[0].alt)
        if not ret:
            return

        for waypoint in self._waypoints:
            # if no marker, just go to waypoint and continue
            if waypoint.aruco_id is None:
                log.info(f"Going to positional waypoint")
                self._drone.goto(waypoint.x, waypoint.y, waypoint.alt)
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

            # capture info about what/where the marker was seen
            location = self._drone.get_location()
            yaw = self._drone.get_attitude()[2]
            latest_marker = detector.get_latest(waypoint.aruco_id)
            marker2 = detector.get_latest(waypoint.aruco2_id)
            if marker2 is not None and marker2.last_seen > marker.last_seen:
                latest_marker = marker2
 
            # fly back to where we saw the marker and point towards it
            self._drone.stop()
            yaw += np.arctan2(latest_marker.location[0], latest_marker.location[2])
            self._drone.goto(location[0], location[1], location[2], yaw=yaw)
            time.sleep(5)  # TODO make not hard coded

            # if marker(s) found, go there
            marker = detector.get_seen(waypoint.aruco_id)
            marker2 = detector.get_seen(waypoint.aruco2_id)
            if not_found:
                log.error("Marker(s) not found in search area")
            elif marker is not None and marker2 is not None:
                log.success(f"Found two markers ({waypoint.aruco_id}, {waypoint.aruco2_id})")
                self._drone.goto((marker.location[2]+marker2.location[2])/2, 
                                     (marker.location[0]+marker.location[0])/2, 
                                     0, relative=True)
            elif marker is not None:
                log.success(f"Found single marker ({waypoint.aruco_id})")
                self._drone.goto(marker.location[2], marker.location[0], 0, relative=True)
            elif marker2 is not None:
                log.success(f"Found single marker ({waypoint.aruco2_id})")
                self._drone.goto(marker2.location[2], marker2.location[0], 0, relative=True)
            else:
                log.error("Marker(s) lost after seen in search area")

            # wait there a bit
            time.sleep(waypoint.wait_time)

        # done with waypoints, go home and land
        self._drone.goto(0, 0, self._waypoints[0].alt)
        self._drone.land()
