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
                 connection_string="udpin:0.0.0.0:14551"):
        """Construct a new Drone object and connect to a mavlink sink/source"""

        # init variables
        self._state = mavutil.mavlink.MAV_STATE_UNINIT
        self._start_time = time.time()

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
                                             timeout=1)

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

    def goto_NEU(self, north, east, alt, relative=False, blocking=True):
        """ Go to a position in NEU coordinates """

        if relative:
            frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
        else:
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        self.connection.mav.set_position_target_local_ned_send(
            int((time.time()-self._start_time)*1000),  # time_boot_ms
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            frame,  # frame
            0b110111111000,  # type_mask (ignore all but positions)
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
        log.info(f"Going to NEU position: {north}, {east}, {alt}")
        while True:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED',
                                             blocking=True)
            if (abs(msg.x - north) < 0.1 and
                    abs(msg.y - east) < 0.1 and
                    abs(msg.z + alt) < 0.1):
                break

        log.info("Drone reached target position")
        return True

    def velocity_NEU(self, north, east, up):
        """ Set the drone's velocity in NED coordinates """

        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        self.connection.mav.set_position_target_local_ned_send(
            int((time.time()-self._start_time)*1000),  # time_boot_ms
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b110111000111,  # type_mask (ignore all but velocities)
            0,  # x
            0,  # y
            0,  # z
            north,  # vx
            east,  # vy
            -up,  # vz
            0,  # afx
            0,  # afy
            0,  # afz
            0,  # yaw
            0)  # yaw_rate

    def stop(self, blocking=True):
        """ Stop the drone's movement """
        self.velocity_NEU(0, 0, 0)

        if blocking:
            while True:
                msg = self.connection.recv_match(type='GLOBAL_POSITION_INT',
                                                 blocking=True)
                if msg.vx < 10 and msg.vy < 10 and msg.vz < 10:  # cm/s
                    break

    def inplace_yaw(self, yaw, blocking=True):
        """ Set the drone's yaw (-180, 180) while maintaining stationary"""
        rad = np.deg2rad(yaw)
        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        self.connection.mav.set_position_target_local_ned_send(
            int((time.time()-self._start_time)*1000),  # time_boot_ms
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b100111000111,  # type_mask (ignore all but velocities)
            0,  # x
            0,  # y
            0,  # z
            0,  # vx
            0,  # vy
            0,  # vz
            0,  # afx
            0,  # afy
            0,  # afz
            rad,  # yaw
            0)

        if blocking:
            log.info(f"Yawing to {yaw} degrees")
            while True:
                msg = self.connection.recv_match(type='ATTITUDE',
                                                 blocking=True)
                if abs(np.arctan2(np.sin(msg.yaw-rad), np.cos(msg.yaw-rad))) < 0.1:
                    break
