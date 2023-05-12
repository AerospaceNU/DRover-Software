#!/usr/bin/env python3

import sys
import random
import numpy as np
from loguru import logger as log
from drover import Drone, MissionController, Waypoint, FiducialDetector, OpenCVCamera, DRoverLEDs, DRoverComms


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

    camera_matrix = np.array([[914.07,  0,      664.35],
                              [0,       917.37, 360.10],
                              [0,       0,      1]], dtype=np.float32)

    dist_coeffs = np.array([.04847, 0.60185, -.00940, .00509, -2.1865], dtype=np.float32)
    cam = OpenCVCamera(camera_matrix, dist_coeffs, width=1920, height=1080, fps=30)

    detector = FiducialDetector(cam, display=False, frames_needed=10, marker_loss_timeout=0.5)
    detector.register_marker_callback(lambda l: leds.flash_color(leds.WHITE))

    # Mission upload and formation
    log.info("Waiting for mission upload...")
    comms = DRoverComms(drone)
    waypoints = comms.get_full_mission()
    log.success(f"Mission uploaded")

    mc = MissionController(drone, waypoints)
    
    # add random offsets to waypoints
    # random.seed(2)
    # for i, wp in enumerate(waypoints):
    #     wp.move_random(i*10)
        
    # run mission
    msg = comms.get_start_signal()
    if msg.param1 == 0:
        mc.fiducial_search_mission(detector, 
                                end_radius=40, 
                                speed=4, 
                                laps=4, 
                                max_dps=10, 
                                search_yaw=180-30)
    else:
        mc.fiducial_search_mission(detector, 
                                   start_radius=msg.param1,
                                   end_radius=msg.param2, 
                                   speed=msg.param3, 
                                   laps=msg.param4, 
                                   max_dps=msg.x, 
                                   search_yaw=180-30)

if __name__ == "__main__":
    drone = Drone()
    try:
        main(drone)
    except Exception as e:
        log.error("Exception caught, RTLing drone...")
        log.exception(e)
        drone.rtl()
    except KeyboardInterrupt as e:
        log.error("Keyboard interrupt, RTLing drone...")
        drone.rtl()
