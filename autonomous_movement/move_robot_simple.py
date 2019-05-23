#!/usr/bin/env python


import dvrk
# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')


# You can home from Python
print("Turn on controller box and home robot")
p.home()
print("aaaaaaaaaaa")