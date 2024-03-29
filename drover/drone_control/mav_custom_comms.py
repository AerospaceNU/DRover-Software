import os
import time
import numpy as np
from threading import Thread, Lock
from pymavlink import mavutil
from loguru import logger as log
from drover import Drone, Waypoint
from pymavlink.dialects.v20 import ardupilotmega as mavlink

""" 
Custom commands are as follows (see handler functions for specifics):
MAV_CMD_USER_1 sets a LAT/LON waypoint, yes I don't care that there is already a protocol for this I'm writing my own (https://mavlink.io/en/services/mission.html)
MAV_CMD_USER_2 sets an NED waypoint (same as above but N/E instead of LAT/LON)
MAV_CMD_USER_3 is the start our mission command
MAV_CMD_USER_4 companion computer reboot signal
MAV_CMD_USER_5 n/a
STATUSTEXT is used for sending log stuff but not custom per say
"""

USER_GCS_COMP_ID = mavlink.MAV_COMP_ID_USER1

class DRoverComms():
    def __init__(self, drone: Drone):
        """ Class that listens for custom MAVLink messages on the drone (ex MAV_CMD_USER_1)"""
        self._drone = drone
        self._waypoints: list[Waypoint] = []
        self._num_waypoints = 0
        self._last_start_msg = None

        drone.add_mavlink_callback("COMMAND_INT", self._handle_command_int)
        
    def _handle_command_int(self, msg: mavlink.MAVLink_command_int_message):
        """ Handle command int messages """
        if msg.command == mavlink.MAV_CMD_USER_1:
            self._handle_user1_cmd(msg)
        elif msg.command == mavlink.MAV_CMD_USER_2:
            self._handle_user2_cmd(msg)
        elif msg.command == mavlink.MAV_CMD_USER_3:
            self._handle_user3_cmd(msg)
        elif msg.command == mavlink.MAV_CMD_USER_4:
            self._handle_user4_cmd(msg)
        else:
            self._acknowledge(msg, mavlink.MAV_RESULT_UNSUPPORTED)
              
    def _handle_user1_cmd(self, msg: mavlink.MAVLink_command_int_message):
        """ lat/lon waypoint callback
            p1: num of waypoints (zero indicates clearing waypoints for new upload attempt)
            p2: waypoint num
            p3: aruco1_id
            p4: aruco2_id
            x:  latitude (int, degE7)
            y:  longitude (int, degE7)
            z:  altitude (int, mm)
            """
        # reset mission if zero waypoints
        if msg.param1 == 0: 
            self._num_waypoints = 0
            self._waypoints = []
            self._acknowledge(msg)
            return
        
        # new mission if nonzero num waypoints and is different than tracked
        if self._num_waypoints != int(msg.param1):
            self._num_waypoints = int(msg.param1)
            self._waypoints = [None]*int(msg.param1)
        
        # add waypoint
        wp = Waypoint(msg.x/1e7, 
                      msg.y/1e7, 
                      msg.z/1e3,
                      use_latlon=True,
                      aruco_id=None if np.isnan(msg.param3) else int(msg.param3),
                      aruco2_id=None if np.isnan(msg.param4) else int(msg.param4))
        self._waypoints[int(msg.param2)] = wp
        
        self._acknowledge(msg)
        log.debug(f"Added waypoint LLA {msg.param2}: {msg.x/1e7}, {msg.y/1e7}, {msg.z/1e3}")
 
    def _handle_user2_cmd(self, msg: mavlink.MAVLink_command_int_message):
        """ north/east waypoint callback
            p1: num of waypoints (zero indicates clearing waypoints for new upload attempt)
            p2: waypoint num
            p3: aruco1_id
            p4: aruco2_id
            x:  northing (int, mm)
            y:  easting (int, mm)
            z:  altitude (int, mm)
            """
        # reset mission if zero waypoints
        if msg.param1 == 0: 
            self._num_waypoints = 0
            self._waypoints = []
            log.debug("Got reset waypoints msg")
            self._acknowledge(msg)
            return
        
        # new mission if nonzero num waypoints and is different than tracked
        if self._num_waypoints != int(msg.param1):
            self._num_waypoints = int(msg.param1)
            self._waypoints = [None]*int(msg.param1)
        
        # add waypoint
        wp = Waypoint(msg.x/1e3, 
                      msg.y/1e3, 
                      msg.z/1e3,
                      use_latlon=False,
                      aruco_id=None if np.isnan(msg.param3) else int(msg.param3),
                      aruco2_id=None if np.isnan(msg.param4) else int(msg.param4))
        self._waypoints[int(msg.param2)] = wp
        
        log.debug(f"Added waypoint NED {msg.param2}: {msg.x/1e3}, {msg.y/1e3}, {msg.z/1e3}")
        self._acknowledge(msg)

    def _handle_user3_cmd(self, msg: mavlink.MAVLink_command_int_message):
        """ start signal command
            p1: float
            p2: float
            p3: float
            p4: float
            x:  int
            y:  int
            z:  int (reset flag)
        """
        self._last_start_msg = msg
        self._acknowledge(msg)  

    def _handle_user4_cmd(self, msg: mavlink.MAVLink_command_int_message):
        """ reboot computer command
            p1: n/a
            p2: n/a
            p3: n/a
            p4: n/a
            x:  n/a
            y:  n/a
            z:  must be 12345
        """
        if msg.z != 12345:
            log.warning("Reset command received without magic 12345")
        self._acknowledge(msg)
        time.sleep(5)
        os.system('sudo shutdown -r now')
        
    def _acknowledge(self, msg: mavlink.MAVLink_command_long_message, result = mavlink.MAV_RESULT_ACCEPTED):
        """ Sends a COMMAND_ACK in response to a command """
        self._drone.mav_conn.mav.command_ack_send(
            msg.command, # command
            result, # result
            0, # progress
            0, # result_param2
            0, # target_system
            0, # target_component
        )
        
    def log(self, text: str, severity=mavlink.MAV_SEVERITY_DEBUG):
        if len(text) > 50:
            log.warning(f"String sent do GCS is truncated ({len(text)})")
            text = text[:50]
        
        self._drone.mav_conn.mav.statustext_send(severity, text.encode('ascii'))    

    def get_full_mission(self):
        """ Waits for a full mission to be uploaded and returns it """
        while self._num_waypoints == 0 or None in self._waypoints:
            self._drone.drain_mavlink_buffer()
            time.sleep(0.01)

        return self._waypoints

    def get_start_signal(self) -> mavlink.MAVLink_command_int_message:
        """ Waits for a new start signal (MAV_CMD_USER_3) """
        self._last_start_msg = None
        while self._last_start_msg == None:
            self._drone.drain_mavlink_buffer()
            time.sleep(0.01)

        return self._last_start_msg

class GCSComms():
    MAV2LOG_SEVERITY = {
        mavlink.MAV_SEVERITY_EMERGENCY:50, # CRITICAL
        mavlink.MAV_SEVERITY_ALERT:50,     # CRITICAL
        mavlink.MAV_SEVERITY_CRITICAL:50,  # CRITICAL
        mavlink.MAV_SEVERITY_ERROR:40,     # ERROR
        mavlink.MAV_SEVERITY_WARNING:30,   # WARNING
        mavlink.MAV_SEVERITY_NOTICE:30,    # WARNING,
        mavlink.MAV_SEVERITY_INFO:20,      # INFO
        mavlink.MAV_SEVERITY_DEBUG:10,     # DEBUG
    }
    
    def __init__(self, 
                 connection_string="tcp:localhost:5762", 
                 baudrate=115200):
        """ Class that allows for sending custom MAVLink messages (ex MAV_CMD_USER_1) """
        self._last_heartbeat = 0
        self._lock = Lock()
        
        self._mav_conn: mavutil.mavfile = mavutil.mavlink_connection(
                                            connection_string,
                                            baud=baudrate,
                                            dialect="ardupilotmega",
                                            autoreconnect=True,
                                            source_component=USER_GCS_COMP_ID)
        
        self.listener_thread = Thread(target=self._run, daemon=True)
        self.listener_thread.start()
        
    def _run(self):
        while True:
            # clear buffer
            self._lock.acquire()
            msg = self._mav_conn.recv_msg()
            self._lock.release()
            if msg is None:
                continue
            
            # check for log messages
            type = msg.get_type()
            if type == "STATUSTEXT":
                log.log(self.MAV2LOG_SEVERITY[msg.severity], msg.text)

            # send heartbeat
            if time.time()-self._last_heartbeat > 0.5:
                self._mav_conn.mav.heartbeat_send(
                    mavlink.MAV_TYPE_GCS,
                    mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavlink.MAV_STATE_ACTIVE)
                self._last_heartbeat = time.time()

    def send_command_int(self, command, param1:float=0,
                          param2:float=0, param3:float=0,
                          param4:float=0, x:int=0,
                          y:int=0, z:int=0,
                          wait_ack=False):
        """ Send a command to the companion computer """
        self._lock.acquire()
        self._mav_conn.mav.command_int_send(
            1,  # target_system
            mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,  # target_component
            0, # frame
            command,  # command
            0,  # current
            0, # autocontinue
            param1,
            param2,
            param3,
            param4,
            int(x),
            int(y),
            int(z))

        if wait_ack:
            ack = self._mav_conn.recv_match(
                                type='COMMAND_ACK',
                                condition=f'COMMAND_ACK.command=={command}',
                                blocking=True, timeout=2)
            self._lock.release()
            if ack is None:
                return False
            return ack.result == mavlink.MAV_RESULT_ACCEPTED
        
        self._lock.release()
        return True

    def send_waypoints(self, waypoints: list[Waypoint]):
        """ Sends waypoints to DRover """
        ret = self.send_command_int(mavlink.MAV_CMD_USER_2, wait_ack=True)
        if not ret:
            log.error(f"No ack for MAV_CMD_USER_1/2. Try re-uploading")
            
        for i, wp in enumerate(waypoints):
            if wp.use_latlon:
                ret = self.send_command_int(
                    mavlink.MAV_CMD_USER_1,
                    len(waypoints),
                    i,
                    wp.aruco_id if wp.aruco_id is not None else float('nan'),
                    wp.aruco2_id if wp.aruco2_id is not None else float('nan'),
                    wp.x*1e7,
                    wp.y*1e7,
                    wp.alt*1e3,
                    wait_ack=True
                )
            else:
                ret = self.send_command_int(
                    mavlink.MAV_CMD_USER_2,
                    len(waypoints),
                    i,
                    wp.aruco_id if wp.aruco_id is not None else float('nan'),
                    wp.aruco2_id if wp.aruco2_id is not None else float('nan'),
                    wp.x*1e3,
                    wp.y*1e3,
                    wp.alt*1e3,
                    wait_ack=True
                )                
            if not ret:
                log.error(f"No ack from WP {i}. Try re-uploading")

    def send_start_signal(self, 
                          param1:float=0.0,
                          param2:float=0.0,
                          param3:float=0.0,
                          param4:float=0.0,
                          x:int=0,
                          y:int=0,
                          z:int=0,):
        """ Sends the start signal """
        ret = self.send_command_int(
            mavlink.MAV_CMD_USER_3,
            param1,
            param2,
            param3,
            param4,
            int(x),
            int(y),
            int(z),
            wait_ack=True
        )                
        if not ret:
            log.error(f"No ack from start signal. GLHF")

    def send_reboot(self):
        ret = self.send_command_int(
            mavlink.MAV_CMD_USER_4,
            0,
            0,
            0,
            0,
            0,
            0,
            int(12345),
            wait_ack=True
        )                
        if not ret:
            log.error(f"No ack from reboot signal. GLHF")

    def wait_heartbeat(self):
        with self._lock:
            self._mav_conn.wait_heartbeat()
