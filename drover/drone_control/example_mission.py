#!/usr/bin/env python3

from drone import Drone

drone = Drone(log_level="INFO")

drone.arm_takeoff(altitude=2)

drone.goto_NEU(5, 0, 2)

drone.land()

drone.wait_disarmed()
