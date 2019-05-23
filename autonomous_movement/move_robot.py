#!/usr/bin/env python
import time
import rospy
import threading
import math
import sys
import csv
import datetime
import numpy
import tty,termios
import PyKDL
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

import dvrk

import xml.etree.ElementTree as ET

# changes done by ritesh

class potentiometer_calibration:

    def __init__(self, robot_name):
        self._robot_name = robot_name
        self._serial_number = ""
        self._data_received = False # use pots to make sure the ROS topics are OK
        self._last_potentiometers = []
        self._last_joints = []
        ros_namespace = '/dvrk/' + self._robot_name
        rospy.Subscriber(ros_namespace +  '/io/analog_input_pos_si', JointState, self.pot_callback)
        rospy.Subscriber(ros_namespace +  '/io/joint_position', JointState, self.joints_callback)
        # callbacks are for getting position and pot values

    def pot_callback(self, data):
        self._last_potentiometers[:] = data.position
        self._data_received = True

    def joints_callback(self, data):
        self._data_received = True
        self._last_joints[:] = data.position


    def run(self, calibrate, filename):
        nb_joint_positions = 20 # number of positions between limits
        nb_samples_per_position = 500 # number of values collected at each position
        total_samples = nb_joint_positions * nb_samples_per_position
        samples_so_far = 0

        sleep_time_after_motion = 0.5 # time after motion from position to position to allow potentiometers to stabilize
        sleep_time_between_samples = 0.01 # time between two samples read (potentiometers)

        d2r = math.pi / 180.0 # degrees to radians
        r2d = 180.0 / math.pi # radians to degrees

        # Looking in XML assuming following tree structure
        # config > Robot> Actuator > AnalogIn > VoltsToPosSI > Scale = ____   or   Offset = ____

        xmlVoltsToPosSI = {}

        tree = ET.parse(filename)
        root = tree.getroot()
        robotFound = False
        stuffInRoot = root.getchildren()
        for index in range(len(stuffInRoot)):
            if stuffInRoot[index].tag == "Robot":
                currentRobot = stuffInRoot[index]
                if currentRobot.attrib["Name"] == self._robot_name:
                    self._serial_number = currentRobot.attrib["SN"]
                    xmlRobot = currentRobot
                    print "Succesfully found robot \"", currentRobot.attrib["Name"], "\", Serial number: ", self._serial_number, " in XML file"
                    robotFound = True
                else:
                    print "Found robot \"", currentRobot.attrib["Name"], "\", while looking for \"", self._robot_name, "\""

        if robotFound == False:
            sys.exit("Robot tree could not be found in xml file")

        # look for all VoltsToPosSI
        stuffInRobot = xmlRobot.getchildren()
        for index in range(len(stuffInRobot)):
            child = stuffInRobot[index]
            if child.tag == "Actuator":
                actuatorId = int(child.attrib["ActuatorID"])
                stuffInActuator = child.getchildren()
                for subIndex in range(len(stuffInActuator)):
                    subChild = stuffInActuator[subIndex]
                    if subChild.tag == "AnalogIn":
                        stuffInAnalogIn = subChild.getchildren()
                        for subSubIndex in range(len(stuffInAnalogIn)):
                            subSubChild = stuffInAnalogIn[subSubIndex]
                            if subSubChild.tag == "VoltsToPosSI":
                                xmlVoltsToPosSI[actuatorId] = subSubChild

        # set joint limits and number of axis based on arm type, using robot name
        if ("").join(list(currentRobot.attrib["Name"])[:-1]) == "PSM": #checks to see if the robot being tested is a PSM
            arm_type = "PSM"
            lower_joint_limits = [-60.0 * d2r, -30.0 * d2r, 0.005, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  30.0 * d2r, 0.235,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r]
            nb_axis = 7 #number of joints being tested
        elif currentRobot.attrib["Name"] == "MTML":
            arm_type = "MTM"
            lower_joint_limits = [-15.0 * d2r, -10.0 * d2r, -10.0 * d2r, -180.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 35.0 * d2r,  20.0 * d2r,  10.0 * d2r,   80.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif currentRobot.attrib["Name"] == "MTMR":
            arm_type = "MTM"
            lower_joint_limits = [-30.0 * d2r, -10.0 * d2r, -10.0 * d2r,  -80.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 15.0 * d2r,  20.0 * d2r,  10.0 * d2r,  180.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif currentRobot.attrib["Name"] == "ECM":
            arm_type = "ECM"
            lower_joint_limits = [-60.0 * d2r, -40.0 * d2r,  0.005, -80.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  40.0 * d2r,  0.230,  80.0 * d2r]
            nb_axis = 4


        if arm_type == "PSM":
            this_arm = dvrk.psm(self._robot_name)
        else:
            this_arm = dvrk.arm(self._robot_name)

        # Check that everything is working
        time.sleep(2.0) # to make sure some data has arrived
        if not self._data_received:
            print "It seems the console for ", self._robot_name, " is not started or is not publishing the IO topics"
            print "Make sure you use \"rosrun dvrk_robot dvrk_console_json\" with the -i option"
            sys.exit("Start the dvrk_console_json with the proper options first")

        # print "The serial number found in the XML file is: ", self._serial_number
        # print "Make sure the dvrk_console_json is using the same configuration file.  Serial number can be found in GUI tab \"IO\"."
        # ok = raw_input("Press `c` and [enter] to continue\n")
        # if ok != "c":
        #     sys.exit("Quitting")

        ######## scale calibration
        # now = datetime.datetime.now()
        # now_string = now.strftime("%Y-%m-%d-%H:%M")
        
        # messages
        # raw_input("To start with some initial values, you first need to \"home\" the robot.  When homed, press [enter]\n")

        # if arm_type == "PSM":
        #     raw_input("Since you are calibrating a PSM, make sure there is no tool inserted.  Please remove tool or calibration plate if any and press [enter]\n")
        # if arm_type == "ECM":
        #     raw_input("Since you are calibrating an ECM, remove the endoscope and press [enter]\n")
        # raw_input("The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed\n")        
        # move to the home position from the start
        # this_arm.home()

        if arm_type == "PSM":
            this_arm.move_jaw(0.0, blocking = False)
            this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        elif arm_type == "MTM":
            this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        elif arm_type == "ECM":
            this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0]))

        time.sleep(2.0)

        print(this_arm.get_current_position())
        
        print(PyKDL.Vector(0.0, 0.05, 0.0))

        this_arm.move(PyKDL.Vector(0.0, 0.0, -0.05))

        print(this_arm.get_current_position())
        print(this_arm.get_desired_position())

        # if arm_type == "PSM":
        #     this_arm.move_jaw(30.0*d2r, blocking = False)
        #     this_arm.move_joint(numpy.array([30.0*d2r, 15.0*d2r, 0.2, 30.0*d2r, 30.0*d2r, 30.0*d2r]))
        # elif arm_type == "MTM":
        #     this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        # elif arm_type == "ECM":
        #     this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0]))

        # time.sleep(1.0)

        # # at the end, return to home position
        # if arm_type == "PSM":
        #     this_arm.move_jaw(0.0*d2r, blocking = False)
        #     this_arm.move_joint(numpy.array([0.0*d2r, 0.0*d2r, 0.0, 0.0*d2r, 0.0*d2r, 0.0]))
        # elif arm_type == "MTM":
        #     this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        # elif arm_type == "ECM":
        #     this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0]))

## Main ##
# if __name__ == '__main__':
#     if (len(sys.argv) != 4):
#         print sys.argv[0] + ' requires three arguments, i.e. "scales"/"offsets", name of dVRK arm and file name.  Always start with scales calibration.'
#     else:
#         if (sys.argv[1] == 'offsets') or (sys.argv[1] == 'scales'):
#             calibration = potentiometer_calibration(sys.argv[2])
#             calibration.run(sys.argv[1], sys.argv[3])
#         else:
#             print sys.argv[0] + ', first argument must be either scales or offsets.  You must start with the scale calibration'