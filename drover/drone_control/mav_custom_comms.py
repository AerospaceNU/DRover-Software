import time
from threading import Thread
from pymavlink import mavutil
from loguru import logger as log
from drover import Drone, Waypoint
from pymavlink.dialects.v20 import ardupilotmega as mavlink

""" 
Custom commands are as follows (see handler functions for specifics):
MAV_CMD_USER_1 sets a LAT/LON waypoint, yes I don't care that there is already a protocol for this I'm writing my own (https://mavlink.io/en/services/mission.html)
MAV_CMD_USER_2 sets an NED waypoint (same as above but N/E instead of LAT/LON)
MAV_CMD_USER_3 is the start our mission command
MAV_CMD_USER_4 n/a
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

        drone.add_mavlink_callback("MAV_CMD_USER_1", self._handle_user1_cmd)
        drone.add_mavlink_callback("MAV_CMD_USER_2", self._handle_user2_cmd)
        
    def _handle_user1_cmd(self, msg: mavlink.MAVLink_command_int_message):
        """ lat/lon waypoint callback
            p1: num of waypoints (zero indicates clearing waypoints for new upload attempt)
            p2: waypoint num
            p3: aruco1_id
            p4: aruco2_id
            p5: latitude (int, degE7)
            p6: longitude (int, degE7)
            p7: altitude (int, mm)
            """
        # reset mission if zero waypoints
        if msg.param1 == 0: 
            self._num_waypoints = 0
            self._waypoints = []
            self._acknowledge(msg)
            return
        
        # new mission if nonzero num waypoints and is different than tracked
        if self._num_waypoints != msg.param1:
            self._num_waypoints = msg.param1
            self._waypoints = [None]*msg.param1
        
        # add waypoint
        wp = Waypoint(msg.x/1e7, 
                      msg.y/1e7, 
                      msg.z/1e3,
                      use_latlon=True,
                      aruco_id=int(msg.param3),
                      aruco2_id=int(msg.param4))
        self._waypoints[msg.param2] = wp
        
        self._acknowledge(msg)
        log.debug(f"Added waypoint LLA {msg.param2}: {msg.x/1e7}, {msg.y/1e7}, {msg.z/1e3}")
 
    def _handle_user2_cmd(self, msg: mavlink.MAVLink_command_int_message):
        """ north/east waypoint callback
            p1: num of waypoints (zero indicates clearing waypoints for new upload attempt)
            p2: waypoint num
            p3: aruco1_id
            p4: aruco2_id
            p5: northing (int, mm)
            p6: easting (int, mm)
            p7: altitude (int, mm)
            """
        # reset mission if zero waypoints
        if msg.param1 == 0: 
            self._num_waypoints = 0
            self._waypoints = []
            self._acknowledge(msg)
            return
        
        # new mission if nonzero num waypoints and is different than tracked
        if self._num_waypoints != msg.param1:
            self._num_waypoints = msg.param1
            self._waypoints = [None]*msg.param1
        
        # add waypoint
        wp = Waypoint(msg.x/1e3, 
                      msg.y/1e3, 
                      msg.z/1e3,
                      use_latlon=False,
                      aruco_id=int(msg.param3),
                      aruco2_id=int(msg.param4))
        self._waypoints[msg.param2] = wp
        
        self._acknowledge(msg)
        log.debug(f"Added waypoint NED {msg.param2}: {msg.x/1e7}, {msg.y/1e7}, {msg.z/1e3}")
        
    def _acknowledge(self, msg: mavlink.MAVLink_command_long_message):
        """ Sends a COMMAND_ACK in response to a command """
        self._drone.mav_conn.mav.command_ack_send(
            msg.command, # command
            mavlink.MAV_RESULT_ACCEPTED, # result
            0, # progress
            0, # result_param2
            msg.get_srcSystem(), # target_system
            msg.get_srcComponent(), # target_component
        )
        
    def log(self, text: str, severity=mavlink.MAV_SEVERITY_DEBUG):
        if len(text) > 50:
            log.warning(f"String sent do GCS is truncated ({len(text)})")
            text = text[:50]
        
        self._drone.mav_conn.mav.statustext_send(severity, text)    

    def get_full_mission(self):
        """ Waits for a full mission to be uploaded and returns it """
        while self._num_waypoints == 0 or None in self._waypoints:
            self._drone.drain_mavlink_buffer()
        
        return self._waypoints

class GCSComms():
    def __init__(self, 
                 connection_string="udpin:0.0.0.0:14552", 
                 baudrate=115200):
        """ Class that allows for sending custom MAVLink messages (ex MAV_CMD_USER_1) """
        self.mav_conn: mavutil.mavfile = mavutil.mavlink_connection(
                                            connection_string,
                                            baud=baudrate,
                                            dialect="ardupilotmega",
                                            autoreconnect=True,
                                            source_component=USER_GCS_COMP_ID)
        
        self.listener_thread = Thread(target=self._run, daemon=True)
        self.listener_thread.start()
        
    def _run(self):
        while True:
            msg = self.mav_conn.recv_msg()
            if msg is None:
                continue
            
            type = msg.get_type()
            if type == "STATUSTEXT":
                log.log(msg.severity, msg.text)

    def send_command_int(self, command, param1:float=0,
                          param2:float=0, param3:float=0,
                          param4:float=0, x:int=0,
                          y:int=0, z:int=0,
                          wait_ack=False):
        """ Send a command to the companion computer """
        self.mav_conn.mav.command_int_send(
            0,  # target_system
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
            ack = self.mav_conn.recv_match(
                                type='COMMAND_ACK',
                                condition=f'COMMAND_ACK.command=={command}',
                                blocking=True, timeout=2)
            if ack is None:
                return False
            return ack.result == mavlink.MAV_RESULT_ACCEPTED
        return True

    def send_waypoints(self, waypoints: list[Waypoint]):
        """ Sends waypoints to DRover """
        ret = self.send_command_int(mavlink.MAV_CMD_USER_2)
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

