#!/usr/bin/env python


import dvrk
import PyKDL
import time
# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')


# You can home from Python
p.move_joint_one(0.2, 2)
time.sleep(1.0)
p.move_joint_one(0.0, 2)
time.sleep(1.0)
a = PyKDL.Vector(0.1, 0.0, 0.0)
p.move(a, interpolate=True)