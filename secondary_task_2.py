#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import ntplib
import time
import datetime
import numpy as np 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
import random 

class image_converter:

	def __init__(self, secondaryTime = 5, file= None):

		self.windowName = "Image Window"
		self.image_pub1 = rospy.Publisher("modified_display_left",Image, queue_size=5)
		self.image_pub2 = rospy.Publisher("modified_display_right",Image, queue_size=5)
		self.image_pub1_compressed = rospy.Publisher("modified_display_left/compressed" ,CompressedImage, queue_size=5)
		self.image_pub2_compressed = rospy.Publisher("modified_display_right/compressed",CompressedImage, queue_size=5)
		self.bridge = CvBridge()
		self.image_sub_left  = rospy.Subscriber("/juan_cam/right/image_raw", Image, self.left_callback)
		self.image_sub_right = rospy.Subscriber("/juan_cam/left/image_raw", Image, self.right_callback)
		self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/camera", Joy, self.pedal_callback)

		self.misalignment = 75
		self.fontSize = 1.4
		self.message =  "Targets: "
		self.timerStr = "Timer: 00:00"
		self.scoreStr = "Score: 000"
		self.score = 0
		self.alpha = 0.22
		self.target = random.sample(range(min(secondaryTime,10)), 2)

		self.initTime = time.time()
		self.secondaryTime = secondaryTime
		self.turnSecondaryTask = False
		self.lastActivation = 0

		#Blink a green rectangle on screen to indicate the user the secondary task is starting
		self.notifyUser= False

		#Kernel used to blurr images when secondary task is active
		self.smoothingKernel = np.ones((5,5),np.float32)/25

		#File to write timestamps
		self.file = file


	def update(self):
		
		currentTime = time.time()
		secondsCounter = int((currentTime - self.initTime))
		seconds = secondsCounter % 60
		minutes = int(secondsCounter / 60)
		self.timerStr = "Timer: {:02d}:{:02d}".format(minutes,seconds)

		if secondsCounter % self.secondaryTime == 0:
			if secondsCounter != self.lastActivation:
				self.turnSecondaryTask = not self.turnSecondaryTask
				self.scoreStr = "Score: {:3d}".format(self.score)
				self.target = random.sample(range(1,min(self.secondaryTime,10)), 2)
				
				self.lastActivation = secondsCounter
				self.file.write("{:.9f} {}\n".format(time.time(),self.turnSecondaryTask))

				if self.turnSecondaryTask:
					self.message  = "Targets {:d}, {:d}".format(self.target[0],self.target[1])
				else:
					self.message = ""

				for i in range(1):
					self.notifyUser = True
					time.sleep(0.4)
					self.notifyUser = False
					time.sleep(0.4)

	def modifyImageAndPublish(self,cv_image, misalignment=0, publisherId=1):

		publisher = self.image_pub1 if publisherId == 1 else self.image_pub2
		compressedPublisher = self.image_pub1_compressed if publisherId == 1 else self.image_pub2_compressed
		
		#Blur the image if the secondary task is on
		if self.turnSecondaryTask and False:
			cv_image = cv2.filter2D(cv_image,-1,self.smoothingKernel)

		overlay = cv_image.copy()
		
		cv2.putText(overlay, self.message,(10+misalignment, 30), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
		cv2.putText(overlay, self.timerStr+"  "+self.scoreStr,(10+misalignment, 75), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
		
		if self.notifyUser:
			color = (0, 255, 0) if self.turnSecondaryTask else (0, 0, 255)
			cv2.rectangle(overlay, (0, 0), (cv_image.shape[1],cv_image.shape[0]), color, -1)

		cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

		try:
			publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

		#### Create and Publish Compressed Image ####
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
		compressedPublisher.publish(msg)

	def left_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.modifyImageAndPublish(cv_image, misalignment=0, publisherId=1)

	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.modifyImageAndPublish(cv_image, misalignment=self.misalignment, publisherId=2)
	
	def pedal_callback(self,data):

		if self.turnSecondaryTask:
			if data.buttons[0]:
				pedalTimestamp = data.header.stamp.secs + data.header.stamp.nsecs *10**(-9)
				secondsCounter = int((pedalTimestamp - self.initTime))
				secondsFirstDigit = secondsCounter % 10 
				
				print(data.buttons)
				print(pedalTimestamp)
				print(secondsCounter)
				
				if any([secondsFirstDigit == x for x in self.target]):
					self.score += 2
					self.scoreStr = "Score: {:3d}".format(self.score)

def main():
	#Create File to save Timestamps
	timeStamp = createTimeStamp()
	file = open("./data/"+timeStamp+"S1.txt",'w')

	#Communicate to NTP server
	ntp,localTime  = ntpClient.request('europe.pool.ntp.org', version=3),time.time()
	file.write("Initial computer time: {:.9f}\n".format(localTime))
	file.write("Initial NTP time:      {:.9f}\n".format(ntp.tx_time))
	file.write("NTP offset:            {:.9f}\n".format(ntp.tx_time))
	file.write("##DATA##\n")
	file.write("timeStamp secondary_task_status\n")

	raw_input("Press any key to start the data collection")
	print("Starting Da vinci Operation...")
	rospy.init_node('image_converter')
	ic = image_converter(file=file, secondaryTime = 20)
	
	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.2)
			ic.update()

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		file.close()
		cv2.destroyAllWindows()
		print("Shutting down")

def createTimeStamp():
	ts = time.time()
	return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H.%M.%S_')

ntpClient = ntplib.NTPClient()

if __name__ == '__main__':
	print("Initializing node")
	main()