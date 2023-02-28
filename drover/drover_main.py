#!/usr/bin/env python3

import time
from drover import Drone

drone = Drone(log_level="INFO")

drone.arm_takeoff(altitude=2)

drone.inplace_yaw(135)
drone.inplace_yaw(-135)
drone.inplace_yaw(0)

drone.goto_NEU(5, 0, 2)

times_start = time.time()
while time.time() - times_start < 5:
    drone.velocity_NEU(-1, 0, 0)
    time.sleep(0.1)  # 10Hz

drone.stop()

drone.land()

drone.wait_disarmed()
