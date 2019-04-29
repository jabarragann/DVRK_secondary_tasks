#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np 

class image_converter:

	def __init__(self, secondaryTime = 5):

		self.windowName = "Image Window"
		self.image_pub1 = rospy.Publisher("modified_display_left",Image, queue_size=5)
		self.image_pub2 = rospy.Publisher("modified_display_right",Image, queue_size=5)
		self.bridge = CvBridge()
		self.image_sub_left  = rospy.Subscriber("/juan_cam/right/image_raw", Image, self.left_callback)
		self.image_sub_right = rospy.Subscriber("/juan_cam/left/image_raw", Image, self.right_callback)

		self.misalignment = 75
		self.fontSize = 1.4
		self.message =  "Secondary Task: on"
		self.timerStr = "Timer: 00:00"
		self.scoreStr = "Score: 000"
		self.alpha = 0.22

		self.initTime = time.time()
		self.secondaryTime = secondaryTime
		self.turnSecondaryTask = False
		self.lastActivation = 0

		#Blink a green rectangle on screen to indicate the user the secondary task is starting
		self.notifyUser= False

		#Kernel used to blurr images when secondary task is active
		self.smoothingKernel = np.ones((5,5),np.float32)/25

		
	def update(self):
		
		currentTime = time.time()
		secondsCounter = int((currentTime - self.initTime))
		seconds = secondsCounter % 60
		minutes = int(secondsCounter / 60)
		self.timerStr = "Timer: {:02d}:{:02d}".format(minutes,seconds)

		if secondsCounter % self.secondaryTime == 0:
			if secondsCounter != self.lastActivation:
				self.turnSecondaryTask = not self.turnSecondaryTask
				self.message = "Secondary Task: {}".format("On" if self.turnSecondaryTask else "Off")
				self.lastActivation = secondsCounter

				for i in range(1):
					self.notifyUser = True
					time.sleep(0.4)
					self.notifyUser = False
					time.sleep(0.4)

	def modifyImageAndPublish(self,cv_image, misalignment=0, publisherId=1):

		publisher = self.image_pub1 if publisherId == 1 else self.image_pub2
		
		#Blur the image if the secondary task is on
		if self.turnSecondaryTask:
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

	def left_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.modifyImageAndPublish(cv_image, misalignment=0, publisherId=1)

		# #Blur the image if the secondary task is on
		# if self.turnSecondaryTask:
		# 	cv_image = cv2.filter2D(cv_image,-1,self.smoothingKernel)

		# overlay = cv_image.copy()
		
		# cv2.putText(overlay, self.message,(10, 30), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
		# cv2.putText(overlay, self.timerStr+"  "+self.scoreStr,(10, 75), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
		
		# if self.notifyUser:
		# 	color = (0, 255, 0) if self.turnSecondaryTask else (0, 0, 255)
		# 	cv2.rectangle(overlay, (0, 0), (cv_image.shape[1],cv_image.shape[0]), color, -1)

		# cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

		# try:
		# 	self.image_pub1.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		# except CvBridgeError as e:
		# 	print(e)

	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.modifyImageAndPublish(cv_image, misalignment=self.misalignment, publisherId=2)
		
		# overlay = cv_image.copy()
		
		# cv2.putText(overlay, self.message,(10+self.misalignment, 30), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)
		# cv2.putText(overlay, self.timerStr+"  "+self.scoreStr,(10+self.misalignment, 75), cv2.FONT_HERSHEY_SIMPLEX, self.fontSize, (0, 0, 255), 3)

		# if self.notifyUser:
		# 	cv2.rectangle(overlay, (0, 0), (cv_image.shape[1],cv_image.shape[0]), (0, 255, 0), -1)

		# cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

		# try:
		# 	self.image_pub2.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		# except CvBridgeError as e:
		# 	print(e)		

def main():
	rospy.init_node('image_converter')
	ic = image_converter()
	
	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.2)
			ic.update()

	except KeyboardInterrupt:
		print("Shutting down")

	cv2.destroyAllWindows()



if __name__ == '__main__':
	print("Initializing node")
	main()