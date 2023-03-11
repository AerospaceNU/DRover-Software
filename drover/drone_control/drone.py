#!/usr/bin/env python3

import sys
import time
import numpy as np
from loguru import logger as log
from threading import Thread
from pymavlink import mavutil
from typing import Dict, Union

# mavutil reference: https://mavlink.io/en/mavgen_python
# MAVLink messages: https://mavlink.io/en/messages/common.html
# ArduPilot: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html


class Drone():
    def __init__(self,
                 connection_string="udpin:0.0.0.0:14551",
                 max_speed=500,
                 max_accel=50):
        """Construct a new Drone object and connect to a mavlink sink/source"""

        # init variables
        self._state = mavutil.mavlink.MAV_STATE_UNINIT
        self._start_time = time.time()
        self._max_speed = max_speed
        self._max_accel = max_accel

        # setup vehicle communication connection
        # https://mavlink.io/en/mavgen_python/#setting_up_connection
        log.info(f"Connecting to drone on {connection_string}")
        self.connection: mavutil.mavfile = mavutil.mavlink_connection(
                                                  connection_string,
                                                  dialect="ardupilotmega",
                                                  autoreconnect=True)

        # start thread for sending heartbeats
        self._main_thread = Thread(target=self._run, daemon=True)
        self._main_thread.start()

        # wait for a heartbeat from the drone (aka it is connected)
        self.wait_heartbeat()
        log.info(f"Drone connected (system {self.connection.target_system} "
                 f"component {self.connection.target_component})")

        # set stream rates to be faster
        self.set_stream_rate( 4, mavutil.mavlink.MAV_DATA_STREAM_ALL)
        self.set_stream_rate(15, mavutil.mavlink.MAV_DATA_STREAM_POSITION)

        # configure some params
        self.param_set("WPNAV_SPEED", self._max_speed)
        self.param_set("WPNAV_ACCEL", self._max_accel)

        # set the system status to active
        self._state = mavutil.mavlink.MAV_STATE_ACTIVE

##################
# Helper functions
##################

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
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

        return True

    def set_state(self, state):
        """ Set the state of the drone """
        self._state = state
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            self._state)

        log.info("Drone state set to " + str(state))

    def _run(self):
        """ Continuously send heartbeats to the drone at 2Hz """
        # "Generally it should be sent from the same thread as
        # all other messages. This is in order to ensure that the heartbeat
        # is only published when the thread is healthy."
        # Oops - Ian

        while True:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                self._state)

            time.sleep(0.5)

##################
# Wait functions
##################

    def wait_heartbeat(self):
        """ Wait for a heartbeat from the drone """
        self.connection.recv_match(type='HEARTBEAT', blocking=True)

    def wait_armable(self):
        """ Wait for the drone to be armable """
        # wait for pre-arm checks to pass
        # https://mavlink.io/en/messages/common.html#SYS_STATUS
        sys_good_health = False
        while not sys_good_health:
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True)
            sys_good_health = (msg.onboard_control_sensors_present
                               & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)

    def wait_disarmed(self):
        """ Wait for the drone to be disarmed """
        # check the armed bit in the heartbeat
        # https://mavlink.io/en/messages/common.html#HEARTBEAT
        while (self.connection.recv_match(
                type='HEARTBEAT',
                blocking=True).base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            pass

##################
# Logic functions
##################

    def is_moving(self, min_speed=5):
        """ Checks if the drone is moving (speed is cm/s) """
        msg = self.connection.recv_match(type='GLOBAL_POSITION_INT',
                                            blocking=True)

        if (abs(msg.vx) > min_speed or 
            abs(msg.vy) > min_speed or 
            abs(msg.vz) > min_speed):  # cm/s
            return True
        return False

    def at_location_NEU(self, north, east, up=None, tolerance=1.0):
        msg = self.connection.recv_match(type='LOCAL_POSITION_NED',
                                            blocking=True)
        if up is None:
            return abs(msg.x - north) < 0.1 and abs(msg.y - east) < 0.1
        else:
            return (abs(msg.x - north) < 0.1 and
                    abs(msg.y - east) < 0.1 and
                    abs(msg.z + up) < 0.1)

##################
# Config functions
##################

    def set_guided_mode(self):
        """ Set the drone to guided mode """
        # TODO consider MAV_CMD_NAV_GUIDED_ENABLE
        # https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html
        return self.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                param2=4,
                wait_ack=True)

    def rtl(self):
        """ Set the drone to RTL mode """
        return self.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
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
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1, wait_ack=True)

        if not armed:
            log.error("Failed to arm drone")
            return False

        if blocking:
            log.info(f"Drone armed. Taking off to {altitude}m")

        # takeoff
        taking_off = self.send_command_long(
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
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

    def set_stream_rate(self, hz, stream=mavutil.mavlink.MAV_DATA_STREAM_ALL):
        """ Set the stream rate of data from the drone """
        # NOTE: mavproxy and the GCS will override this
        # run `set streamrate -1` to disable in mavproxy, and look in GCS settings
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            stream, # stream
            hz,     # rate
            1)      # start/stop

##################
# Motion functions
##################

    def land(self, blocking=True):
        """ Land the drone """

        self.stop(blocking=True)

        ack = self.send_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
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

    def goto_NEU(self, north, east, alt, 
                 relative=False, yaw=None, yaw_rate=None, 
                 blocking=True, stop_function=None):
        """ Go to a position in NEU coordinates """

        if relative:
            frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
        else:
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

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

        # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        self.connection.mav.set_position_target_local_ned_send(
            int((time.time()-self._start_time)*1000),  # time_boot_ms
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            frame,  # frame
            type_mask,  # type_mask
            north,  # x
            east,  # y
            -alt,  # z
            0,  # vx
            0,  # vy
            0,  # vz
            0,  # afx
            0,  # afy
            0,  # afz
            0,  # yaw
            0)  # yaw_rate

        if not blocking:
            return True

        # wait for the drone to reach the target position
        log.info(f"Going to NEU position: {north:.2f}, {east:.2f}, {alt:.2f}")
        while not self.at_location_NEU(north, east, alt):
            time.sleep(0.001)
            if stop_function is not None and stop_function():
                log.debug("Stopped heading to position due to stop function")
                self.stop()
                return False

        log.debug("Drone reached target position")
        return True

    def velocity_NEU(self, north, east, up, yaw=None, yaw_rate=None):
        """ Set the drone's velocity in NED coordinates (and yaw in radians) """

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
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
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

    def stop(self, blocking=True, accel=200):
        """ Hard stop the drone's movement (accel cm/s/s) """
        self.param_set("WPNAV_ACCEL", accel)
        self.velocity_NEU(0, 0, 0, yaw_rate=0)

        if blocking:
            while self.is_moving():
                time.sleep(0.001)

        self.param_set("WPNAV_ACCEL", self._max_accel)

    def circle_NEU(self, north, east, up, radius, yaw=0.0,
                   laps=1.0, speed=1.0, ccw=False, spiral_out_per_lap=0.0, 
                   stop_on_complete=True, stop_function=None):
        """ Perform a circle around provided point (blocking, yaw in degrees) """

        if speed/radius > 1/5:
            log.warning(f"Circle speed/radius ratio over 1/5 ({speed:.1f}/{radius:.1f})")

        location_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        location = np.array([location_msg.x, location_msg.y])

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
            ret = self.goto_NEU(*closest_start_point, up, stop_function=stop_function)
            if not ret:
                return False

        if laps >= 0.5:
            log.debug(f"Starting circle around ({north:.2f}, {east:.2f}, {up:.2f}) with radius {radius:.2f}m")
        location_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        location = np.array([location_msg.x, location_msg.y])
        angle_traveled_offset = angle_traveled = last_angle = 0
        
        # loop until enough laps have completed
        while angle_traveled_offset+angle_traveled < laps*2*np.pi:
            # check stop contidtion
            if stop_function is not None and stop_function():
                log.debug("Stopping circle due to stop function")
                if stop_on_complete:
                    self.stop()
                return False

            # get current location
            location_msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            location = np.array([location_msg.x, location_msg.y])

            # calculate the circle's tangent vector at the current location
            towards_circle = circle_center-location
            dist_to_circle = np.sqrt(np.sum(towards_circle**2))
            normal_to_circle = towards_circle/dist_to_circle
            normal_tangent = np.array([-normal_to_circle[1], normal_to_circle[0]])

            # calculate the radius for spirals
            spiral_radius = radius + (angle_traveled_offset+angle_traveled)/(2*np.pi)*spiral_out_per_lap

            # calculate velocity vector along the tangent and with a component to correct drifting away from the center
            if not ccw:
                normal_tangent *= -1
            correction_vec = normal_to_circle*(dist_to_circle-spiral_radius)
            velocity_vec = normal_tangent*speed+correction_vec

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

        return True