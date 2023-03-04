#!/usr/bin/env python3

import sys
from loguru import logger as log
from drover import Drone, MissionController, Waypoint

# init pretty logging
logger_format = (
    "<green>{time:HH:mm:ss.SSS}</green> | "
    "<level>{level: <8}</level> | "
    "<level>{message}</level>"
)
log.remove()
log.add(sys.stderr, level="INFO", format=logger_format)


def main(drone: Drone):
    # define waypoints
    waypoints_NEU = [
        Waypoint(  0, -40+1, 2),
        Waypoint( 30, -60+2, 2),
        Waypoint( 60, -20+3, 2),
        Waypoint( 60+4,  50, 2),
        Waypoint(-40,  40+5, 2)
    ]

    # init drone
    drone.param_set("WPNAV_SPEED", 500)
    drone.param_set("WPNAV_ACCEL", 50)

    # init mission controller
    mc = MissionController(drone, waypoints_NEU)

    # run mission
    mc.run_mission()


if __name__ == "__main__":
    drone = Drone()
    try:
        main(drone)
    except Exception as e:
        log.error("Exception caught, RTLing drone...")
        log.exception(e)
        drone.rtl()
    except KeyboardInterrupt:
        log.error("Keyboard interrupt, RTLing drone...")
        drone.rtl()
