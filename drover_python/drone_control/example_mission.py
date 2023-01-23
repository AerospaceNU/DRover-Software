#!/usr/bin/env python3

from drone import Drone

drone = Drone(log_level="INFO")

drone.arm_takeoff()

drone.land()

drone.wait_disarmed()
