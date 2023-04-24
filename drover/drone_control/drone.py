#!/usr/bin/env python3

import time
import navpy
import numpy as np
from threading import Thread
from typing import Dict, Union
from loguru import logger as log
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

# mavutil reference: https://mavlink.io/en/mavgen_python
# MAVLink messages: https://mavlink.io/en/messages/common.html
# ArduPilot: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html


class Drone():
    def __init__(self,
                 connection_string="udpin:0.0.0.0:14551",
                 baudrate=115200,
                 max_speed=750,
                 max_accel=50):
        """Construct a new Drone object and connect to a mavlink sink/source"""

        # init variables
        self._state = mavlink.MAV_STATE_UNINIT
        self._start_time = time.time()
        self._max_speed = max_speed
        self._max_accel = max_accel
        self._prev_orbit_args = None 
        self._last_vehicle_heartbeat = 0
        self._global_origin = None
        self._vehicle_status: mavlink.MAVLink_sys_status_message = None

        # setup vehicle communication connection
        # https://mavlink.io/en/mavgen_python/#setting_up_connection
        log.info(f"Connecting to drone on {connection_string}")
        self.connection: mavutil.mavfile = mavutil.mavlink_connection(
                                                  connection_string,
                                                  baud=baudrate,
                                                  dialect="ardupilotmega",
                                                  autoreconnect=True)
        self.connection.message_hooks.append(self._mav_msg_handler)

        # start thread for sending heartbeats
        self._main_thread = Thread(target=self._run, daemon=True)
        self._main_thread.start()

        # wait for a heartbeat from the drone (aka it is connected)
        self.wait_heartbeat()
        log.info(f"Drone connected (system {self.connection.target_system} "
                 f"component {self.connection.target_component})")

        # set stream rates to be faster
        self.set_stream_rate(   4, mavlink.MAV_DATA_STREAM_ALL)
        self.set_message_rate(  1, mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
        self.set_message_rate( 15, mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        self.set_message_rate( 15, mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        self.set_message_rate( 15, mavlink.MAVLINK_MSG_ID_ATTITUDE)

        # configure some params
        self.param_set("WPNAV_SPEED", self._max_speed)
        self.param_set("WPNAV_ACCEL", self._max_accel)
        self.clear_mission()

        # set the system status to active
        self._state = mavlink.MAV_STATE_ACTIVE

    def _run(self):
        """ Continuously send heartbeats to the drone at 2Hz """
        # "Generally it should be sent from the same thread as
        # all other messages. This is in order to ensure that the heartbeat
        # is only published when the thread is healthy."
        # Oops - Ian

        while True:
            self.connection.mav.heartbeat_send(
                mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                self._state)

            time.sleep(0.5)

    def _mav_msg_handler(self, mav, msg):
        """ Function called for every new mavlink message 
            (called within self.connection.recv_match()) 
        """
        
        type = msg.get_type()
        if type == "HEARTBEAT":
            self._last_vehicle_heartbeat = time.time()
        elif type == "SYS_STATUS":
            self._vehicle_status = msg
            # TODO handle leaving guided mode
        elif type == "GPS_GLOBAL_ORIGIN":
            self._global_origin = (msg.latitude/1e7, msg.longitude/1e7, msg.altitude/1e3)                

        
###################
# Mavlink functions
###################

    def drain_mavlink_buffer(self):
        """ Drain the mavlink buffer """
        while self.connection.recv_match() is not None:
            pass
        log.debug("Drained mavlink buffer")

    def print_n_messages(self, n):
        """ Print the next n messages from the drone (blocking) """
        self.drain_mavlink_buffer()

        for _ in range(n):
            print(self.connection.recv_match(blocking=True).to_dict())

    def send_command_long(self, command, param1=0,
                          param2=0, param3=0,
                          param4=0, param5=0,
                          param6=0, param7=0,
                          wait_ack=False):
        """ Send a command to the drone """

        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            command,  # command
            0,  # confirmation
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7)

        log.debug(f"Sent command {command} to drone")

        if wait_ack:
            ack = self.connection.recv_match(
                                type='COMMAND_ACK',
                                condition=f'COMMAND_ACK.command=={command}',
                                blocking=True, timeout=1)
            if ack is None:
                log.debug(f"Failed to receive ack for command {command}")
                return False
            log.debug(f"Received ack for command {command} "
                      f"with result {ack.result}")
            return ack.result == mavlink.MAV_RESULT_ACCEPTED

        return True

    def set_state(self, state):
        """ Set the state of the drone """
        self._state = state
        self.connection.mav.heartbeat_send(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            self._state)

        log.info("Drone state set to " + str(state))

    def set_stream_rate(self, hz, stream=mavlink.MAV_DATA_STREAM_ALL):
        """ Set the stream rate of data from the drone """
        # https://ardupilot.org/dev/docs/mavlink-requesting-data.html
        # NOTE: mavproxy and the GCS will override this
        # run `set streamrate -1` to disable in mavproxy, and look in GCS settings
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            stream, # stream
            hz,     # rate
            1)      # start/stop

    def set_message_rate(self, hz, msg_id):
        """ Set the rate of a specific message from the drone """
        # https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
        self.send_command_long(
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            param1=msg_id,
            param2=1000000 / hz,
            wait_ack=True)

##################
# Logic functions
##################

    def wait_global_origin(self):
        """ Wait for the global origin to be set"""
        if self._global_origin is None:
            log.debug("Waiting for global origin")
            self.connection.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True)

    def wait_heartbeat(self):
        """ Wait for a heartbeat from the drone """
        self.connection.recv_match(type='HEARTBEAT', blocking=True)

    def wait_armable(self):
        """ Wait for the drone to be armable """
        # wait for pre-arm checks to pass and for a GPS fix
        # https://mavlink.io/en/messages/common.html#SYS_STATUS
        self.connection.wait_gps_fix()
        sys_good_health = False
        while not sys_good_health:
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True)
            sys_good_health = (msg.onboard_control_sensors_present
                               & mavlink.MAV_SYS_STATUS_PREARM_CHECK)

    def wait_disarmed(self):
        """ Wait for the drone to be disarmed """
        # check the armed bit in the heartbeat
        # https://mavlink.io/en/messages/common.html#HEARTBEAT
        while (self.connection.recv_match(
                type='HEARTBEAT',
                blocking=True).base_mode
                & mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            pass

    def is_moving(self, min_speed=5, min_yawspeed=0.1):
        """ Checks if the drone is moving (speed is cm/s, rot in rad/s) """
        pos_msg = self.connection.recv_match(type='GLOBAL_POSITION_INT',
                                            blocking=True)

        attitude_msg = self.connection.recv_match(type='ATTITUDE', blocking=True)

        if (abs(pos_msg.vx) > min_speed or 
            abs(pos_msg.vy) > min_speed or 
            abs(pos_msg.vz) > min_speed or
            abs(attitude_msg.yawspeed) > min_yawspeed):
            return True
        return False

    def at_location(self, x, y, z=None, use_latlon=False, tolerance=1.0):
        """ Returns if we are `tolerance` meters from the provided location"""
        if use_latlon:
            x, y = self.latlon_to_NEU(x, y)
        
        location = self.get_location()

        if z is None:
            return (abs(location[0] - x) < 0.1 and 
                    abs(location[1] - y) < 0.1)
        else:
            return (abs(location[0] - x) < 0.1 and
                    abs(location[1] - y) < 0.1 and
                    abs(location[2] - z) < 0.1)

##################
# Data functions
##################

    def latlon_to_NEU(self, lat, lon, alt=None):
        """ Convert a latitude and longitude to meters northing and easting """
        self.wait_global_origin()
        altitude = 0 if alt is None else alt        
        north, east, down = navpy.lla2ned(lat, lon, altitude, 
                                          self._global_origin[0],
                                          self._global_origin[1],
                                          self._global_origin[2])
        if alt is None:
            return north, east
        else:
            return north, east, -down

    def NEU_to_latlon(self, north, east, alt=None):
        """ Convert a north and east to latitude and longitude """
        self.wait_global_origin()
        altitude = 0 if alt is None else alt
        lat, lon, altitude = navpy.ned2lla((north, east, -altitude), 
                                           self._global_origin[0],
                                           self._global_origin[1],
                                           self._global_origin[2])
        if alt is None:
            return lat, lon
        else:
            return lat, lon, altitude

    def get_location(self, use_latlon=False):
        """ Get current location in NEU frame (north, east, up) or (lat, lon, MSL) """
        if not use_latlon:
            location_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            return np.array([location_msg.x, location_msg.y, -location_msg.z])

        location_msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        return np.array([location_msg.lat/1e7, location_msg.lon/1e7, location_msg.alt/1e3])
    
    def get_velocity_NEU(self):
        """ Get current velocity in NEU frame (north, east, up) """
        location_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        return np.array([location_msg.vx, location_msg.vy, -location_msg.vz])

    def get_attitude(self):
        """ Get current attitude in euler angles (roll, pitch, yaw) """
        attitude_msg = self.connection.recv_match(type='ATTITUDE', blocking=True)
        return np.array([attitude_msg.roll, attitude_msg.pitch, attitude_msg.yaw])

    def rel_to_abs_NEU(self, north, east, up, use_heading=True):
        """ Convert relative coordinates to absolute coordinates """
        current_location = self.get_location()
        
        if use_heading:
            heading = self.get_attitude()[2]

            rotation_matrix = np.array([[np.cos(heading), -np.sin(heading)],
                                        [np.sin(heading),  np.cos(heading)]])

            north_rel, east_rel = rotation_matrix @ np.array([north, east])
            return current_location + np.array([north_rel, east_rel, up]) 

        return current_location + np.array([north, east, up])


##################
# Config functions
##################

    def upload_mission_latlon(self, waypoints: list[tuple[float, float,float]], terrain_alt=True):
        """ Upload a mission to the drone in the format of a list of (lat, lon, alt)"""

        # first waypoint must be home/start location of sorts
        self.wait_global_origin()        
        cur_lat, cur_lon, cur_alt = self.get_location(use_latlon=True)
        
        if terrain_alt:
            frame = mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            waypoints.insert(0, (cur_lat, cur_lon, 
                                 (cur_alt-self._global_origin[2])+waypoints[0][2]))
        else:
            frame = mavlink.MAV_FRAME_GLOBAL_INT
            waypoints.insert(0, (cur_lat, cur_lon, 
                                 self._global_origin[2]+waypoints[0][2]))

        log.warning(f"{len(waypoints)}")
        self.connection.waypoint_count_send(len(waypoints))
        for i in range(len(waypoints)):
            msg = self.connection.recv_match(type=['MISSION_REQUEST'], blocking=True)
            log.debug(f"Sending waypoint {msg.seq}...")
            
            self.connection.mav.mission_item_int_send(
                self.connection.target_system,  # target_system
                self.connection.target_component,  # target_component
                msg.seq, # seq
                frame, # frame
                mavlink.MAV_CMD_NAV_WAYPOINT,
                int(False), # current
                int(True), # autocontinue
                0, 0, 0, 0, # param1-4
                int(waypoints[msg.seq][0]*1e7),
                int(waypoints[msg.seq][1]*1e7),
                waypoints[msg.seq][2],
            )

        log.info("Waypoints uploaded")

    def set_home_rally_point(self):
        """ Sets a rally point at the current location and sets RALLY_INCL_HOME to false """
        self.add_rally_point()
        # https://ardupilot.org/copter/docs/parameters.html#rally-incl-home-rally-include-home
        self.param_set("RALLY_INCL_HOME", 0)

    def clear_mission(self):
        """ Clear all mission items on the drone """
        self.connection.waypoint_clear_all_send()

    def add_rally_point(self, lat=None, lon=None, alt=None, current_location=True):
        """ Adds a rally point either at a specified location or current location"""
        if not current_location and None in [lat, lon, alt]:
            log.error("None passed to add_rally_point")
            
        if current_location:
            lat, lon, alt = self.get_location(use_latlon=True)
        
        self.send_command_long(mavlink.MAVLINK_MSG_ID_RALLY_POINT, 
                               param5=lat,
                               param6=lon,
                               param7=alt)

    def set_guided_mode(self):
        """ Set the drone to guided mode """
        # TODO consider MAV_CMD_NAV_GUIDED_ENABLE
        # https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html
        return self.send_command_long(
                mavlink.MAV_CMD_DO_SET_MODE,
                param1=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                param2=4,
                wait_ack=True)

    def rtl(self):
        """ Set the drone to RTL mode """
        return self.send_command_long(
                mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                wait_ack=True)

    def arm_takeoff(self, altitude=2.5, blocking=True):
        """ Arm the drone """

        self.wait_armable()

        # go into guided mode so we can send position commands
        if not self.set_guided_mode():
            log.error("Failed to set guided mode")
            return False

        # send command to arm the drone (set param2 to 21196 to force arming)
        # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
        armed = self.send_command_long(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1, wait_ack=True)

        if not armed:
            log.error("Failed to arm drone")
            return False

        if blocking:
            log.info(f"Drone armed. Taking off to {altitude}m")

        # takeoff
        taking_off = self.send_command_long(
                            mavlink.MAV_CMD_NAV_TAKEOFF,
                            param7=altitude,
                            wait_ack=True)

        if not taking_off:
            log.error("Failed to takeoff")
            return False

        if blocking:
            # wait for the drone to reach the target altitude
            while True:
                msg = self.connection.recv_match(type='GLOBAL_POSITION_INT',
                                                 blocking=True)
                if msg.relative_alt / 1000 > altitude * 0.95:
                    break
            log.info("Drone reached target takeoff altitude")

        return True

    def param_set(self, parm_name, parm_value, param_type=None, retries=3):
        """ Wrapper for parameter send function"""

        for _ in range(retries):
            self.connection.param_set_send(parm_name, parm_value, param_type)
            msg = self.connection.recv_match(type='PARAM_VALUE',
                                             blocking=True,
                                             condition=f'PARAM_VALUE.param_id=="{parm_name}"',
                                             timeout=0.5)

            if msg is not None:
                return True

        log.error(f"Failed to set {parm_name} to {parm_value}")
        return False


##################
# Motion functions
##################

    def land(self, blocking=True):
        """ Land the drone """

        self.stop(blocking=True)

        ack = self.send_command_long(
                    mavlink.MAV_CMD_NAV_LAND,
                    wait_ack=True)

        if not ack:
            log.error("Failed to land drone")
            return False

        if not blocking:
            return True

        log.info("Landing drone")
        while True:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT',
                                             blocking=True)
            if msg.relative_alt / 1000 < 0.1:
                break

        log.info("Drone landed")
        return True

    def goto(self, x, y, z, use_latlon=False, above_terrain=False,
                 relative=False, yaw=None, yaw_rate=None, 
                 blocking=True, stop_function=None):
        """ Fly to a provided location

        Args:
            x (float): north or latitude (see use_latlon)
            y (float): east or longitude (see use_latlon)
            z (float): altitude
            use_latlon (bool, optional): Use global latitude/longitude rather than NEU. Defaults to False.
            above_terrain (bool, optional): If use_latlong, this makes altitude relative to terrain. Defaults to False.
            relative (bool, optional): If not use_latlon, this makes NEU movements relative. Defaults to False.
            yaw (float, optional): absolute yaw while flying to location. Defaults to None.
            yaw_rate (float, optional): yaw (spin) rate while flying to location. Defaults to None.
            blocking (bool, optional): Causes function to block and wait for arrival at location. Defaults to True.
            stop_function (function, optional): Function that breaks blocking if it returns True. Defaults to None.

        Returns:
            bool: Successfully at location
        """
        
        if use_latlon and relative:
            log.error("Cannot use relative latlon movement")
            return False

        if blocking:
            log.info(f"Going to NEU {'relative' if relative else 'position'}: {x:.2f}, {y:.2f}, {z:.2f}")

        # if relative, convert to absolute target position
        # this was done rather than using the MAV_FRAME_LOCAL_OFFSET_NED frame
        # so we could keep track of the absolute target position
        if relative:
            x, y, z = self.rel_to_abs_NEU(x, y, z)

        # ignore yaw if arguments not provided
        ignore_yaw       = 0b010000000000 
        ignore_yaw_rate  = 0b100000000000
        ignore_vel_accel = 0b000111111000
        if yaw is not None:
            type_mask = ignore_vel_accel | ignore_yaw_rate
        elif yaw_rate is not None:
            type_mask = ignore_vel_accel | ignore_yaw
        else:
            type_mask = ignore_vel_accel | ignore_yaw | ignore_yaw_rate

        # send the command
        # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        if use_latlon:
            if above_terrain:
                frame = mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
            else:
                frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                
            # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
            self.connection.mav.set_position_target_global_int_send(
                int((time.time()-self._start_time)*1000),  # time_boot_ms
                self.connection.target_system,  # target_system
                self.connection.target_component,  # target_component
                frame,  # frame
                x,  # x
                y,  # y
                z,  # z
                0,  # vx
                0,  # vy
                0,  # vz
                0,  # afx
                0,  # afy
                0,  # afz
                yaw if yaw is not None else 0,  # yaw
                yaw_rate if yaw_rate is not None else 0)  # yaw_rate
                
        else:
            # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
            self.connection.mav.set_position_target_local_ned_send(
                int((time.time()-self._start_time)*1000),  # time_boot_ms
                self.connection.target_system,  # target_system
                self.connection.target_component,  # target_component
                mavlink.MAV_FRAME_LOCAL_NED,  # frame
                type_mask,  # type_mask
                x,  # x
                y,  # y
                -z,  # z
                0,  # vx
                0,  # vy
                0,  # vz
                0,  # afx
                0,  # afy
                0,  # afz
                yaw if yaw is not None else 0,  # yaw
                yaw_rate if yaw_rate is not None else 0)  # yaw_rate

        if not blocking:
            return True

        # wait for the drone to reach the target position
        while not self.at_location(x, y, z, use_latlon=use_latlon):
            time.sleep(0.001)
            if stop_function is not None and stop_function():
                log.debug("Stopped heading to position due to stop function")
                self.stop()
                return False

        log.debug("Drone reached target position")
        return True

    def velocity_NEU(self, north, east, up, yaw=None, yaw_rate=None, body_offset=False):
        """ Set the drone's velocity in NED coordinates (and yaw in radians) """

        if body_offset:
            frame = mavlink.MAV_FRAME_BODY_NED
        else:
            frame = mavlink.MAV_FRAME_LOCAL_NED

        # ignore yaw if arguments not provided
        ignore_yaw       = 0b010000000000 
        ignore_yaw_rate  = 0b100000000000
        ignore_pos_accel = 0b000111000111

        if yaw is not None:
            type_mask = ignore_pos_accel | ignore_yaw_rate
        elif yaw_rate is not None:
            type_mask = ignore_pos_accel | ignore_yaw
        else:
            type_mask = ignore_pos_accel | ignore_yaw | ignore_yaw_rate


        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        self.connection.mav.set_position_target_local_ned_send(
            int((time.time()-self._start_time)*1000),  # time_boot_ms
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            frame,  # frame
            type_mask,  # type_mask 
            0,  # x
            0,  # y
            0,  # z
            north,  # vx
            east,  # vy
            -up,  # vz
            0,  # afx
            0,  # afy
            0,  # afz
            yaw if yaw is not None else 0,  # yaw
            yaw_rate if yaw_rate is not None else 0)  # yaw_rate

    def stop(self, blocking=True, accel=100):
        """ Hard stop the drone's movement (accel cm/s/s) """
        self.param_set("WPNAV_ACCEL", accel)
        self.velocity_NEU(0, 0, 0, yaw_rate=0)

        if blocking:
            while self.is_moving():
                time.sleep(0.001)

        self.param_set("WPNAV_ACCEL", self._max_accel)

    def orbit_NEU(self, north, east, up, radius, yaw=0.0,
                  laps=1.0, speed=1.0, ccw=False, spiral_out_per_lap=0.0, 
                  stop_on_complete=True, max_dps=10, stop_function=None):
        """ Perform an orbit around provided point (blocking, yaw in degrees).
            Returns true if the orbit was completed, false if it was stopped.
        """

        location = self.get_location()[:2]

        # calculate closest point from the drone on the circle
        circle_center = np.array([north, east])
        towards_circle = circle_center-location
        dist_to_circle = np.sqrt(np.sum(towards_circle**2))
        start_normal_to_circle = towards_circle/dist_to_circle
        closest_start_point = circle_center - start_normal_to_circle*radius

        # fly to circle if not already on it
        # TODO set yaw here too so we dont double back if in the circle
        if np.sqrt(np.sum((location-closest_start_point)**2)) > 1:
            log.debug(f"Flying to closest point on circle ({location[0]:.2f}, {location[1]:.2f}) -> ({closest_start_point[0]:.2f}, {closest_start_point[1]:.2f})")
            ret = self.goto(*closest_start_point, up, stop_function=stop_function)
            if not ret:
                self._prev_orbit_args = (north, east, up, radius, yaw, laps, 
                                           speed, ccw, spiral_out_per_lap, 
                                           stop_on_complete, max_dps, 
                                           stop_function)
                return False

        if laps >= 0.5:
            log.debug(f"Starting circle around ({north:.2f}, {east:.2f}, {up:.2f}) with radius {radius:.2f}m")

        # loop until enough laps have completed
        angle_traveled_offset = angle_traveled = last_angle = 0
        spiral_radius = radius
        while angle_traveled_offset+angle_traveled < laps*2*np.pi:
            # check stop condition
            if stop_function is not None and stop_function():
                log.debug("Stopping circle due to stop function")
                if stop_on_complete:
                    self.stop()
                laps_left = laps-(angle_traveled_offset+angle_traveled)/(2*np.pi)
                self._prev_orbit_args = (north, east, up, spiral_radius, yaw, 
                                           laps_left, speed, ccw, 
                                           spiral_out_per_lap, 
                                           stop_on_complete, max_dps,
                                           stop_function)
                return False

            # calculate the circle's tangent vector at the current location
            location = self.get_location()[:2]
            towards_circle = circle_center-location
            dist_to_circle = np.sqrt(np.sum(towards_circle**2))
            normal_to_circle = towards_circle/dist_to_circle
            normal_tangent = np.array([-normal_to_circle[1], normal_to_circle[0]])
            if not ccw:
                normal_tangent *= -1

            # calculate the radius for spirals
            spiral_radius = radius + (angle_traveled_offset+angle_traveled)/(2*np.pi)*spiral_out_per_lap

            # calculate velocity vector along the tangent and with a component to correct drifting away from the center
            max_dps_speed = np.deg2rad(max_dps)*spiral_radius
            correction_vec = normal_to_circle*(dist_to_circle-spiral_radius)
            velocity_vec = normal_tangent*min(max_dps_speed, speed)+correction_vec

            # calculate yaw wrt tangent of circle, and add yaw offset
            yaw_calculated = None
            if yaw != 0:
                angle_from_north = np.arctan2(-towards_circle[1], -towards_circle[0])
                yaw_calculated = (angle_from_north-np.deg2rad(yaw-90)) % (2 * np.pi) - np.pi

            # calculate angle distance traveled (accounting for wrap around)
            dot = towards_circle[0]*start_normal_to_circle[0] + towards_circle[1]*start_normal_to_circle[1]
            det = towards_circle[0]*start_normal_to_circle[1] - towards_circle[1]*start_normal_to_circle[0]
            angle_traveled = -np.arctan2(det, dot)  
            if abs(angle_traveled-last_angle) > np.pi/2:
                angle_traveled_offset += 2*np.pi
            last_angle = angle_traveled

            # send the velocity and yaw
            self.velocity_NEU(*velocity_vec, 0, yaw=yaw_calculated)
        
        # done with circle, so cancel veloctiy
        if stop_on_complete:
            self.stop()

        self._prev_orbit_args = None
        return True

    def resume_orbit(self):
        """ Resumes the last orbit """
        if self._prev_orbit_args is None:
            return False
        
        else:
            return self.orbit_NEU(*self._prev_orbit_args)
            

