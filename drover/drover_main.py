#!/usr/bin/env python3

import time
from drover import Drone

TARGET_ALT = 2

drone = Drone(log_level="INFO")

flying = drone.arm_takeoff(altitude=TARGET_ALT)
if not flying:
    exit(1)

waypoints_NEU = [
    [  0, -20, TARGET_ALT],
    [ 15, -30, TARGET_ALT],
    [ 30, -10, TARGET_ALT],
    [ 30,  25, TARGET_ALT],
    [-20,  20, TARGET_ALT],
]

for wp in waypoints_NEU:
    drone.goto_NEU(*wp)
    time.sleep(2)

drone.goto_NEU(0, 0, TARGET_ALT)
drone.land()
