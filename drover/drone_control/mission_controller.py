
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
    land: bool = True
    use_latlon: bool = False
    aruco_id: int = None
    aruco2_id: int = None
    marker_avoid_dist: float = 1.0

    def move_random(self, dist):
        """ Moves waypoint roughly `dist` m in random direction """
        angle = random.uniform(0, 2*np.pi)
        dx = dist*np.cos(angle)
        dy = dist*np.sin(angle)
        if self.use_latlon:
            # approximate transforms
            r_earth = 6371000.0
            self.y += (dx / r_earth) * (180 / np.pi) / np.cos(self.x * 180/np.pi)
            self.x += (dy / r_earth) * (180 / np.pi)
        else:
            self.x += dx
            self.y += dy


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

    def goto_simple_waypoint(self, waypoint):
        """ Handles a simple goto waypoint without marker """    
        self._drone.goto(waypoint.x, waypoint.y, waypoint.alt, use_latlon=waypoint.use_latlon)
        time.sleep(waypoint.wait_time)
        if waypoint.land:
            self._drone.land()
            time.sleep(waypoint.wait_time)        
            self._drone.arm_takeoff(waypoint.alt)

    def search_for_arucos(self, detector, waypoint, start_radius, end_radius, speed, laps, max_dps, yaw):
        """ Search for aruco markers"""
        # determine if we are searching for a single marker or a pair
        if waypoint.aruco2_id is None:
            stop_func = lambda: (detector.get_seen(waypoint.aruco_id) is not None)
        else:
            stop_func = lambda: (detector.get_seen(waypoint.aruco_id) is not None or 
                                 detector.get_seen(waypoint.aruco2_id) is not None)
        # perform search pattern
        not_found = self._drone.orbit(waypoint.x, waypoint.y, waypoint.alt, 
                            start_radius, speed=speed, laps=laps, yaw=yaw,
                            spiral_out_per_lap=(end_radius-start_radius)/laps,
                            stop_on_complete=False, use_latlon=waypoint.use_latlon,
                            max_dps=max_dps, stop_function=stop_func)
        if not_found:
            return False

        # capture info about what/where the marker was seen incase we fly past it
        location = self._drone.get_location()
        yaw = self._drone.get_attitude()[2]
        latest_marker = detector.get_latest(waypoint.aruco_id)
        marker2 = detector.get_latest(waypoint.aruco2_id)
        if latest_marker is None or (marker2 is not None and marker2.last_seen > latest_marker.last_seen):
            latest_marker = marker2
            
        # calculate a location a bit closer to the seen marker 
        # so we ensure we have a good view of it
        target_dist_away = 8
        marker_n, marker_e, alt = self._drone.rel_to_abs_NEU(latest_marker.location[2],latest_marker.location[0], 0)
        ne_displacement = np.array([marker_n-location[0], 
                                    marker_e-location[1]])
        dist_away = np.linalg.norm(ne_displacement)
        backtrack_adjustment = (ne_displacement/dist_away) * target_dist_away
        log.debug(f"Saw marker {dist_away:.2f}m away, backtracking...")
        log.warning(f"{ne_displacement}")
        
        # fly back to close to the marker and point towards it
        self._drone.stop()
        yaw += np.arctan2(latest_marker.location[0], latest_marker.location[2])
        self._drone.goto(location[0]+backtrack_adjustment[0], location[1]+backtrack_adjustment[1], location[2], yaw=yaw)
        time.sleep(3)  # TODO make not hard coded (wait till see for a)
        
        return True

    def goto_seen_marker(self, relative_location, waypoint):
        """ provided a relative marker location, go to it (minus marker_avoid_dist)"""
        xz_displacement = np.array([relative_location[0], relative_location[2]])
        dist_away = np.linalg.norm(xz_displacement)
        goal = (xz_displacement/dist_away) * (dist_away-waypoint.marker_avoid_dist)
        
        self._drone.goto(goal[1], goal[0], 0, relative=True)
        time.sleep(waypoint.wait_time)
        if waypoint.land:
            self._drone.land()
            time.sleep(waypoint.wait_time)        
            self._drone.arm_takeoff(waypoint.alt)


##########
# Missions
##########

    def simple_mission(self):
        """
        Run a mission that simply goes to each waypoint in order
        """
        self._pre_mission()
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self.goto_simple_waypoint(waypoint)

        self._drone.rtl()
        self._drone.wait_disarmed()
        
    def circle_mission(self, radius=10.0, speed=2.0, laps=1, yaw=90):
        """ Run a mission that circles all waypoints """
        self._pre_mission()
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit(waypoint.x, waypoint.y, waypoint.alt, radius, speed=speed, laps=laps, yaw=yaw, use_latlon=waypoint.use_latlon)
            time.sleep(waypoint.wait_time)

        self._drone.rtl()
        self._drone.wait_disarmed()
        
    def spiral_mission(self, end_radius=30.0, start_radius=10, speed=2.0, laps=4):
        """ Run a mission that spirals all waypoints """
        self._pre_mission()
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit(waypoint.x, waypoint.y, waypoint.alt, 
                                   start_radius, speed=speed, laps=laps, yaw=90,
                                   spiral_out_per_lap=(end_radius-start_radius)/laps, 
                                   use_latlon=waypoint.use_latlon)
            time.sleep(waypoint.wait_time)

        self._drone.rtl()
        self._drone.wait_disarmed()

    def fiducial_search_mission(self, detector: FiducialDetector, 
                                start_radius=5, end_radius=20.0, speed=4.0, 
                                laps=4, max_dps=10, search_yaw=0):
        """ Run a mission that searches for aruco markers at waypoints """
        self._pre_mission()
        ret = self._drone.arm_takeoff(self._waypoints[0].alt)
        if not ret:
            return

        for waypoint in self._waypoints:
            # if no marker, just go to waypoint and continue
            if waypoint.aruco_id is None:
                log.info(f"Going to positional waypoint")
                self.goto_simple_waypoint(waypoint)
                continue

            # else if there is a marker, search for it/them
            found = self.search_for_arucos(detector,
                                           waypoint,
                                           start_radius,
                                           end_radius,
                                           speed,
                                           laps,
                                           max_dps,
                                           search_yaw)

            # if marker(s) found, go there
            marker = detector.get_seen(waypoint.aruco_id)
            marker2 = detector.get_seen(waypoint.aruco2_id)
            if not found:
                log.error("Marker(s) not found in search area")
            elif marker is not None and marker2 is not None:
                log.success(f"Found two markers ({waypoint.aruco_id}, {waypoint.aruco2_id})")
                # TODO make good    
                self._drone.goto((marker.location[2]+marker2.location[2])/2, 
                                     (marker.location[0]+marker.location[0])/2, 
                                     0, relative=True)
            elif marker is not None:
                log.success(f"Found single marker ({waypoint.aruco_id})")
                self.goto_seen_marker(marker.location, waypoint)
            elif marker2 is not None:
                log.success(f"Found single marker ({waypoint.aruco2_id})")
                self.goto_seen_marker(marker2.location, waypoint)
            else:
                log.warning("Marker(s) lost after seen in search area")

            # wait there a bit
            time.sleep(waypoint.wait_time)

        # done with waypoints, go home and land
        self._drone.rtl()
        self._drone.wait_disarmed()
