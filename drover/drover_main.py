#!/usr/bin/env python3

import sys
import random
from loguru import logger as log
from drover import Drone, MissionController, Waypoint, FiducialDetector, SimCamera, DRoverLEDs


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
    waypoints = [
        # Waypoint(  0,  40, 2, wait_time=2),
        Waypoint(-35.36299172424103, 149.16589752767177, 2, use_latlon=True, aruco_id=1, wait_time=2),
        Waypoint(-35.36272135187389, 149.16545744182363, 2, use_latlon=True, aruco_id=2, wait_time=2),
        Waypoint(-35.36272135082222, 149.16468729544093, 2, use_latlon=True, aruco_id=3, wait_time=2),
        Waypoint(-35.36362259779146, 149.16479731146183, 2, use_latlon=True, aruco_id=4, aruco2_id=5, wait_time=2)
    ]

    # add random offsets
    random.seed(2)
    for i, wp in enumerate(waypoints):
        wp.move_random(i*10)

    # init objects
    mc = MissionController(drone, waypoints)
    leds = DRoverLEDs(drone)
    detector = FiducialDetector(SimCamera(), display=True, frames_needed=10, marker_loss_timeout=0.5)
    detector.register_marker_callback(lambda l: leds.flash_color(leds.WHITE))
    
    # run mission
    # mc.simple_mission()
    # mc.circle_mission(radius=10.0, speed=1.0)
    # mc.spiral_mission()
    mc.fiducial_search_mission(detector, end_radius=40, laps=4, speed=4, max_dps=10, search_yaw=180-30)

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
