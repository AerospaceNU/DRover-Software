#!/usr/bin/env python3

import sys
import time
import random
import numpy as np
from loguru import logger as log
from drover import Drone, MissionController, Waypoint, FiducialDetector, OpenCVCamera, RaspberryPiCamera, SimCamera, DRoverLEDs, DRoverComms


# init pretty logging
logger_format = (
    "<green>{time:HH:mm:ss.SSS}</green> | "
    "<level>{level: <8}</level> | "
    "<level>{message}</level>"
)
log.remove()
log.add(sys.stderr, level="DEBUG", format=logger_format)


# main
def main(drone: Drone):
    # define waypoints
    # waypoints = [
    #     # Waypoint(  0,  40, 2, wait_time=2),
    #     Waypoint(-35.36299172424103, 149.16589752767177, 2, use_latlon=True, aruco_id=1, wait_time=2),
    #     Waypoint(-35.36272135187389, 149.16545744182363, 2, use_latlon=True, aruco_id=2, wait_time=2),
    #     Waypoint(-35.36272135082222, 149.16468729544093, 2, use_latlon=True, aruco_id=3, wait_time=2),
    #     Waypoint(-35.36362259779146, 149.16479731146183, 2, use_latlon=True, aruco_id=4, aruco2_id=5, wait_time=2)
    # ]
    
    # setup
    leds = DRoverLEDs(drone)

    #### 1080 camera
    camera_matrix = np.array([[1496.2,  0,      962.51],
                              [0,       1493.7, 542.75],
                              [0,       0,      1]], dtype=np.float32)

    dist_coeffs = np.array([0.1161, 0.0606, 0, 0, 0], dtype=np.float32)
    cam = RaspberryPiCamera(camera_matrix, dist_coeffs, width=1920, height=1080)
    #### 720 camera
    # camera_matrix = np.array([[835.5,  0,      541.84],
    #                           [0,       834.7, 369.68],
    #                           [0,       0,      1]], dtype=np.float32)

    # dist_coeffs = np.array([0.1397, -0.1348, 0, 0, 0], dtype=np.float32)
    # cam = RaspberryPiCamera(camera_matrix, dist_coeffs, width=1280, height=720)

    # cam = SimCamera()

    detector = FiducialDetector(cam, display=True, frames_needed=3, marker_loss_timeout=0.5)
    detector.register_marker_callback(lambda l: leds.flash_color(leds.WHITE, priority=False))
    detector.disable()
    
    while True:
        # Mission upload and formation
        log.info("Waiting for mission upload...")
        drone.send_statustext("drover: waiting for mission")
        comms = DRoverComms(drone)
        waypoints = comms.get_full_mission()
        log.success(f"Mission uploaded")
        drone.send_statustext("drover: mission uploaded!")

        mc = MissionController(drone, waypoints, leds)
        mc.upload_waypoints()
                    
        # run mission
        msg = comms.get_start_signal()
        if msg.z == int(True):
            continue
            
        drone.send_statustext("drover: starting mission")
        if msg.param1 == 0:
            log.success("Starting with default args...")
            while not mc.spinny_search_mission(detector):
                time.sleep(5)
                
        else:
            log.success("Starting with custom args...")
            while not mc.spinny_search_mission(detector,
                                               spin_dps=msg.param1,
                                               second_ring_distance=msg.param2):
                                               time.sleep(5)
                
        # done (hopefully) with mission so set loiter mode
        drone.wait_disarmed()
        drone.set_loiter_mode()

if __name__ == "__main__":
    # drone = Drone()
    drone = Drone(connection_string="/dev/serial0")
    try:
        main(drone)
    except Exception as e:
        log.error("Exception caught, RTLing drone...")
        drone.send_statustext(f"drover: Error {type(e)}") 
        log.exception(e)
        drone.rtl()
    except KeyboardInterrupt as e:
        log.error("Keyboard interrupt, RTLing drone...")
        log.exception(e)
        drone.rtl()
