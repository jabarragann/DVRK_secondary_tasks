#!/usr/bin/env python


'''
NOTES

dmove methods will increment in a certain amount the position of the robot.
You only need to specify the increment

move methods will move the robot to a new position. 

Joint positions have to specified in radians. I have not try to move the robot to position where he cannot go,
so try to keep the movement of the robot within a safe range.

Be careful when specifying the cartesian values for the robot since weird values will make the arm go to 
weird postures. From what I have tested until now you always have to specify a value in the z axis of 
around -0.1 to be able to set the x and y.

'''
import time
import dvrk
import numpy as np
import PyKDL


# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')


# You can home from Python
print("Turn on controller box, home robot and get robot information")
p.home()

d2r = np.pi / 180.0

# limits of the joint movement
# lower_joint_limits = [-60.0 * d2r, -30.0 * d2r, 0.005, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r]
# upper_joint_limits = [ 60.0 * d2r,  30.0 * d2r, 0.235,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r]

#Move robot to zero position

print("Joints Positions")
print(p.get_current_joint_position())
print("Joints Velocity")
print(p.get_current_joint_velocity())
print("Joints Effort")
print(p.get_current_joint_effort())
print("Get Cartesian position")
print(p.get_current_position())

print("Press c and enter to move the robot using cartesian position. Make sure the robot will no collide")
# p.move(PyKDL.Vector(0.0, 0.0, 0.0))

# key = raw_input()
# if key == 'c':
	#Move to zero position
	# p.move(PyKDL.Vector(0.0, 0.0, 0.0))

while(1):
	key = raw_input()
	if key == 't':
		# move up
		p.dmove(PyKDL.Vector(0.0, 0.0, +0.01))
	if key == 'g':
		# move down
		p.dmove(PyKDL.Vector(0.0, 0.0, -0.01))
	if key == 'a':
		# move left x
		p.dmove(PyKDL.Vector(-0.001, 0.0, 0.0))
	if key == 'd':
		# move right x
		p.dmove(PyKDL.Vector(+0.001, 0.0, 0.0))
	if key == 'w':
		# move back y
		p.dmove(PyKDL.Vector(0.0, +0.001, 0.0))
	if key == 's':
		# move front y
		p.dmove(PyKDL.Vector(0.0, -0.001, 0.0))

	# for moving the joints
	if key == 'o':
		p.open_jaw()
	if key == 'c':
		p.close_jaw()

	if key == 'u':
		p.dmove_joint_one(+2*d2r, 4)
	if key == 'j':
		p.dmove_joint_one(-2*d2r, 4)
	if key == 'h':
		p.dmove_joint_one(-2*d2r, 5)
	if key == 'k':
		p.dmove_joint_one(+2*d2r, 5)
	if key == 'i':
		p.dmove_joint_one(+2*d2r, 3)
	if key == 'l':
		p.dmove_joint_one(-2*d2r, 3)

	if key == 'q':
		break

	print("Cartesian position")
	print(p.get_current_position())
	time.sleep(0.1)
print('The end')