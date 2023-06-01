
"""
Mission Controller for DRover
"""

import time
import random
import numpy as np
from loguru import logger as log
from dataclasses import dataclass
from typing import List
from drover import Drone, FiducialDetector, ArucoMarker, DRoverLEDs

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
    marker_avoid_dist: float = 0.5

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
                 waypoints: List[Waypoint] = None,
                 leds: DRoverLEDs = None, 
                 use_home_rally_point: bool = True):
        """Class handling drone missions

        Args:
            drone (Drone): Drone object
            waypoints (List[Waypoint]): List of waypoints in the mission
            use_home_rally_point (bool, optional): Sets a rally point before taking off. Defaults to True.
        """
        self._waypoints = waypoints
        self._leds = leds
        self._drone = drone
        self._use_home_rally_point = use_home_rally_point

    def set_waypoints(self, waypoints: List[Waypoint]):
        """ (Re)sets the waypoint list """
        self._waypoints = waypoints
        
##########
# Mission helpers
##########

    def _pre_mission(self):
        """ Called at the beginning of every mission """
        # validate waypoints
        if self._waypoints is None:
            # TODO do more sanity checks
            log.error("Waypoint list is NoneType")
            return False

        # rally home point
        if self._use_home_rally_point:
            self._drone.set_home_rally_point()
            # force to only RTL to rally point(s)
            self._drone.param_set("RALLY_INCL_HOME", 0)
        
        # Upload waypoints as a mission for GCS visualization purposes
        self.upload_waypoints()
        return True

    def upload_waypoints(self):
        """ Uploads the mission to the drone for GCS visualization """
        mission_list = []
        for wp in self._waypoints:
            if wp.use_latlon:
                mission_list.append((wp.x, wp.y, wp.alt))
            else:
                lat, lon = self._drone.NEU_to_latlon(wp.x, wp.y)
                mission_list.append((lat, lon, wp.alt))
                
        self._drone.upload_mission_latlon(mission_list)

    def goto_simple_waypoint(self, waypoint, last_waypoint=False):
        """ Handles a simple goto waypoint without marker """    
        self._drone.goto(waypoint.x, waypoint.y, waypoint.alt, use_latlon=waypoint.use_latlon)
        self._drone.send_statustext(f"At positional waypoint")
        if self._leds:
            self._leds.flash_color(DRoverLEDs.GREEN, 5*waypoint.wait_time)
        time.sleep(waypoint.wait_time)
        if waypoint.land:
            self._drone.land()
            time.sleep(waypoint.wait_time)   
            if not last_waypoint:     
                self._drone.arm_takeoff(waypoint.alt)

    def spiral_for_arucos(self, detector, waypoint, start_radius, end_radius, 
                          speed, laps, max_dps, yaw, backtrack_dist=8):
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
        marker_n, marker_e, alt = self._drone.rel_to_abs_NEU(latest_marker.location[2],latest_marker.location[0], 0)
        ne_displacement = np.array([marker_n-location[0], 
                                    marker_e-location[1]])
        dist_away = np.linalg.norm(ne_displacement)
        backtrack_adjustment = (ne_displacement/dist_away) * (dist_away-backtrack_dist)
        log.debug(f"Saw marker {dist_away:.2f}m away, backtracking...")
        
        # fly back to close to the marker and point towards it
        self._drone.stop()
        yaw += np.arctan2(latest_marker.location[0], latest_marker.location[2])
        self._drone.goto(location[0]+backtrack_adjustment[0], location[1]+backtrack_adjustment[1], location[2], yaw=yaw)
        time.sleep(3)  # TODO make not hard coded (wait till see for a)
        
        return True

    def goto_seen_marker(self, relative_location, waypoint, last_waypoint=False):
        """ provided a relative marker location, go to it (minus marker_avoid_dist)"""
        xz_displacement = np.array([relative_location[0], relative_location[2]])
        dist_away = np.linalg.norm(xz_displacement)
        goal = (xz_displacement/dist_away) * (dist_away-waypoint.marker_avoid_dist)
        
        self._drone.goto(goal[1], goal[0], 0, relative=True)
        self._drone.send_statustext(f"drover: at marker {waypoint.aruco_id}")
        if self._leds:
            self._leds.flash_color(DRoverLEDs.GREEN, 5*waypoint.wait_time)
        time.sleep(waypoint.wait_time)
        if waypoint.land:
            self._drone.land()
            time.sleep(waypoint.wait_time)        
            if not last_waypoint: 
                self._drone.arm_takeoff(waypoint.alt)

    def goto_gate(self, marker1, marker2, waypoint, last_waypoint=False):
        # TODO make good    
        self._drone.goto((marker1.location[2]+marker2.location[2])/2, 
                                (marker1.location[0]+marker1.location[0])/2, 
                                0, relative=True)
        self._drone.send_statustext(f"At gate {waypoint.aruco_id}/{waypoint.aruco_id2}")
        if self._leds:
            self._leds.flash_color(DRoverLEDs.GREEN, 5*waypoint.wait_time)
        if last_waypoint:
            self._drone.land()

    def goto_but_watchout(self, detector, x, y, alt, use_latlon, marker1, marker2=None):
        """ GOTO but watch for markers and return True if we found and went to it """
        # define what we are trying to find for stop functions
        stop_func = lambda: (detector.get_seen(marker1) is not None or
                            (marker2 is not None and 
                            detector.get_seen(marker2) is not None))
        # goto loop
        while True:
            not_found = self._drone.goto(x, y, alt, 
                                        use_latlon=use_latlon,
                                        stop_function=stop_func)
            
            # if saw marker go to it and we done here, else continue journey 
            if not_found:
                return False
            else:
                success = self.head_to_marker(detector, marker1, marker2)
                if success:
                    return True
                else:
                    continue

    def head_to_marker(self, detector: FiducialDetector, marker1, marker2, dist_away=1, alt_change_dist=4, final_alt_agl=1.25, fly_speed=0.5, rot_speed=8, center_tolerance=0.1, yolo_lost_advance_dist=0.5):
        """ Gos to a marker by keeping it in sight, doesn't rely on 3d position """
        if marker1 is not None:
            marker_id = marker1
        elif marker2 is not None:
            marker_id = marker2
        else:
            self._drone.send_statustext(f"drover: head to got no markers")
            return False
        
        self._drone.send_statustext(f"drover: heading to marker {marker_id}")
        self._drone.stop()
        time.sleep(2)
        
        # center on marker
        max_losses = 4
        lost_count = 0
        marker_x_pos = 0
        while not (0.5-center_tolerance < marker_x_pos < 0.5+center_tolerance):
            # get marker and handle if not seen
            marker = detector.get_seen(marker_id)
            if marker is None:
                self._drone.stop()
                lost_count += 1
                if lost_count > max_losses:
                    self._drone.send_statustext(f"drover: lost in centering")
                    return False
                time.sleep(1)
                continue
            
            # center
            marker_x_pos = marker.image_location[0]
            yaw_rate = np.deg2rad(rot_speed) * (1 if marker_x_pos > 0.5 else -1)
            self._drone.velocity_NEU(0, 0, 0, yaw_rate=yaw_rate)
            
        self._drone.stop(blocking=False)
        self._drone.send_statustext(f"drover: centered, heading towards")
        
        # head towards marker
        lost_count = 0
        dist = dist_away+alt_change_dist+1
        marker_x_pos = 0
        while dist > dist_away+alt_change_dist:
            # get marker and handle if not seen
            marker = detector.get_seen(marker_id)
            if marker is None:
                self._drone.stop()
                lost_count += 1
                if lost_count > max_losses:
                    self._drone.send_statustext(f"drover: lost in head towards")
                    return False
                time.sleep(1)
                continue

            # center if drifted
            marker_x_pos = marker.image_location[0]
            yaw_rate = np.deg2rad(rot_speed) * (1 if marker_x_pos > 0.5 else -1)
            yaw_rate = 0 if (0.5-center_tolerance < marker_x_pos < 0.5+center_tolerance) else yaw_rate
            
            # head towards
            location_2d = np.array([marker.location[2], marker.location[0]])
            dist = np.linalg.norm(location_2d)
            direction_2d = location_2d/dist
            velocity = direction_2d * fly_speed
            self._drone.velocity_NEU(*velocity, 0, yaw_rate=yaw_rate, body_offset=True)

        self._drone.stop(blocking=False)
        self._drone.send_statustext(f"drover: adjusting alt")

        # adjust altitude
        self._drone.set_altitude_AGL(final_alt_agl)
        time.sleep(3)

        self._drone.send_statustext(f"drover: at alt, fine moving")

        # fine head to marker
        lost_count = 0
        dist = dist_away+1
        marker_x_pos = 0
        while dist > dist_away:
            # get marker and handle if not seen
            marker = detector.get_seen(marker_id)
            if marker is None:
                self._drone.stop()
                lost_count += 1
                if lost_count > max_losses:
                    log.warning("Lost marker in final slow heading towards. Landing")
                    self._drone.send_statustext(f"drover: lost in final heading. shimmy")
                    self._drone.goto(yolo_lost_advance_dist, 0, 0, relative=True, yaw_rate=0)
                    self._drone.send_statustext(f"drover: shimmied, landing")
                    break
                time.sleep(1)
                continue

            # center if drifted
            marker_x_pos = marker.image_location[0]
            yaw_rate = np.deg2rad(rot_speed/2) * (1 if marker_x_pos > 0.5 else -1)
            yaw_rate = 0 if (0.5-center_tolerance < marker_x_pos < 0.5+center_tolerance) else yaw_rate
            
            # head towards
            location_2d = np.array([marker.location[2], marker.location[0]])
            dist = np.linalg.norm(location_2d)
            direction_2d = location_2d/dist
            velocity = direction_2d * fly_speed
            self._drone.velocity_NEU(*velocity, 0, yaw_rate=yaw_rate, body_offset=True)

        self._drone.stop()
        return True

    def signal_success_and_land(self, waypoint):
        """ Flashy lights, log, and lands"""
        if self._leds:
            self._leds.flash_color(DRoverLEDs.GREEN, 5*waypoint.wait_time)
        log.success(f"Found marker {waypoint.aruco_id}")
        self._drone.send_statustext(f"drover: found marker {waypoint.aruco_id}")
        self._drone.land()

##########
# Missions
##########

    def simple_mission(self):
        """
        Run a mission that simply goes to each waypoint in order
        """
        if not self._pre_mission():
            return False
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self.goto_simple_waypoint(waypoint)

        self._drone.rtl()
        self._drone.wait_disarmed()
        return True
        
    def circle_mission(self, radius=10.0, speed=2.0, laps=1, yaw=90):
        """ Run a mission that circles all waypoints """
        if not self._pre_mission():
            return False
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit(waypoint.x, waypoint.y, waypoint.alt, radius, speed=speed, laps=laps, yaw=yaw, use_latlon=waypoint.use_latlon)
            time.sleep(waypoint.wait_time)

        self._drone.rtl()
        self._drone.wait_disarmed()
        return True
        
    def spiral_mission(self, end_radius=30.0, start_radius=10, speed=2.0, laps=4):
        """ Run a mission that spirals all waypoints """
        if not self._pre_mission():
            return False
        self._drone.arm_takeoff(self._waypoints[0].alt)

        for waypoint in self._waypoints:
            self._drone.orbit(waypoint.x, waypoint.y, waypoint.alt, 
                                   start_radius, speed=speed, laps=laps, yaw=90,
                                   spiral_out_per_lap=(end_radius-start_radius)/laps, 
                                   use_latlon=waypoint.use_latlon)
            time.sleep(waypoint.wait_time)

        self._drone.rtl()
        self._drone.wait_disarmed()
        return False

    def fiducial_search_mission(self, detector: FiducialDetector, 
                                start_radius=5, end_radius=30.0, speed=2.0, 
                                laps=6, max_dps=5, search_yaw=180-45):
        """ Run a mission that searches for aruco markers at waypoints """
        if not self._pre_mission():
            return False
        
        ret = self._drone.arm_takeoff(self._waypoints[0].alt)
        if not ret:
            self._drone.send_statustext("drover: Failed arm_takeoff")
            return False

        for i, waypoint in enumerate(self._waypoints):
            last_waypoint = (i == (len(self._waypoints)-1))
            
            # if no marker, just go to waypoint and continue
            if waypoint.aruco_id is None:
                log.info(f"Going to positional waypoint")
                self.goto_simple_waypoint(waypoint, last_waypoint=last_waypoint)
                continue

            # else if there is a marker, search for it/them
            detector.enable()
            for _ in range(2):
                found = self.spiral_for_arucos(detector,
                                            waypoint,
                                            start_radius,
                                            end_radius,
                                            speed,
                                            laps,
                                            max_dps,
                                            search_yaw)
                if found:
                    break
                log.warning("Retrying search pattern")
                self._drone.send_statustext("drover: Retrying search")

            self._drone.send_statustext(f"drover: Found marker {waypoint.aruco_id}")

            # if marker(s) found, go there
            marker = detector.get_seen(waypoint.aruco_id)
            marker2 = detector.get_seen(waypoint.aruco2_id)
            if not found:
                log.error("Marker(s) not found in search area")
                self._drone.send_statustext("Couldnt find marker, RTLing")
                if last_waypoint:
                    self._drone.rtl()
            elif marker is not None and marker2 is not None:
                log.success(f"Found two markers ({waypoint.aruco_id}, {waypoint.aruco2_id})")
                self.goto_gate(marker, marker2, waypoint, last_waypoint=last_waypoint)
            elif marker is not None:
                log.success(f"Found single marker ({waypoint.aruco_id})")
                self.goto_seen_marker(marker.location, waypoint, last_waypoint=last_waypoint)
            elif marker2 is not None:
                log.success(f"Found single marker ({waypoint.aruco2_id})")
                self.goto_seen_marker(marker2.location, waypoint, last_waypoint=last_waypoint)
            else:
                log.warning("Marker(s) lost after seen in search area")

            # wait there a bit
            detector.disable()
            time.sleep(waypoint.wait_time)

        # done with waypoints, land in case
        detector.disable()
        self._drone.land()
        time.sleep(5)
        self._drone.set_loiter_mode()
        return True

    def spinny_search_mission(self, 
                              detector: FiducialDetector, 
                              spin_dps=20,
                              second_ring_distance=16
                              ):
        """ Run a mission that spinny searches for aruco markers at waypoints 
            This search that I made up goes to the waypoint, spins, then does
            a set of spins at 8 points `second_ring_distance` away (first in 
            cardinal directions, then in-between). """
            
        if not self._pre_mission():
            return False
        
        ret = self._drone.arm_takeoff(self._waypoints[0].alt)
        if not ret:
            self._drone.send_statustext("drover: Failed arm_takeoff")
            return False

        # make everything lat/lon
        for wp in self._waypoints:
            if not wp.use_latlon:
                wp.use_latlon = True
                x, y = self._drone.NEU_to_latlon(wp.x, wp.y)
                wp.x = x
                wp.y = y

        # mission time
        for i, waypoint in enumerate(self._waypoints):
            last_waypoint = (i == (len(self._waypoints)-1))
            
            # if no marker, just go to waypoint and continue
            if waypoint.aruco_id is None:
                log.info(f"Going to positional waypoint")
                self.goto_simple_waypoint(waypoint, last_waypoint=last_waypoint)
                continue

            # else if there is a marker, search for it/them
            detector.enable()
            # go to waypoint, watching for markers
            found = self.goto_but_watchout(detector, waypoint.x, waypoint.y, waypoint.alt, 
                                           waypoint.use_latlon,
                                           marker1=waypoint.aruco_id,
                                           marker2=waypoint.aruco2_id)
            if found:
                self.signal_success_and_land(waypoint)
                continue

            # marker wasn't found on our way so lets spinny search
            # define what we are trying to find for spin stop function
            stop_func = lambda: (detector.get_seen(waypoint.aruco_id) is not None or
                                (waypoint.aruco2_id is not None and 
                                detector.get_seen(waypoint.aruco2_id) is not None))
            # create spin locations list (weird numbering enforces alternation)
            spins = [(waypoint.x, waypoint.y)]
            wp_n, wp_e = self._drone.latlon_to_NEU(waypoint.x, waypoint.y)
            for i in [0, 2, 4, 6, 7, 1, 3, 5]:
                spin_n = second_ring_distance * np.cos(i * np.pi/4) + wp_n
                spin_e = second_ring_distance * np.sin(i * np.pi/4) + wp_e
                spin_latlon = self._drone.NEU_to_latlon(spin_n, spin_e)
                spins.append(spin_latlon)

            # goto spinny locations and spinnn
            success = False
            for i, spin_loc in enumerate(spins):
                self._drone.send_statustext(f"drover: goto spinny {i}")
                if i != 0:
                    found = self.goto_but_watchout(detector, *spin_loc, waypoint.alt, 
                                                waypoint.use_latlon,
                                                marker1=waypoint.aruco_id,
                                                marker2=waypoint.aruco2_id)
                    if found:
                        success = True
                        break
                    
                # spin
                not_found = self._drone.spin(spin_dps, stop_function=stop_func)
                if not_found:
                    continue
                # found
                success = self.head_to_marker(detector, waypoint.aruco_id, waypoint.aruco2_id)
                if success:
                    break
                # found but lost so we spun by it probs, so spin back
                not_found = self._drone.spin(-spin_dps/2, stop_function=stop_func)
                if not_found:
                    continue
                success = self.head_to_marker(detector, waypoint.aruco_id, waypoint.aruco2_id)
                # if we saw it again and got there, nice. Otherwise just continue
                if success:
                    break
                continue
                
            # Handle the L
            if not success:
                if last_waypoint:
                    self._drone.rtl()
                log.warning("Failed to find marker")
                self._drone.send_statustext("drover: exhausted search area")
                continue
            
            # land and wait there a bit
            time.sleep(waypoint.wait_time)
            self.signal_success_and_land(waypoint)
            detector.disable()
            time.sleep(waypoint.wait_time)
            if not last_waypoint:
                self._drone.arm_takeoff(waypoint.alt)

        # done with waypoints, land in case
        detector.disable()
        self._drone.wait_disarmed()
        self._drone.set_loiter_mode()
        return True
