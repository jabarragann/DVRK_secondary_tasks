#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub1 = rospy.Publisher("/juan_cam/left/inverted",Image,queue_size=1)
    self.image_pub2 = rospy.Publisher("/juan_cam/right/inverted",Image,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub1 = rospy.Subscriber("/juan_cam/left/image_raw",Image,self.callback1)
    self.image_sub2 = rospy.Subscriber("/juan_cam/right/image_raw",Image,self.callback2)

  def callback1(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    inv_image = np.array(cv_image)
    inv_image = np.flipud(inv_image)
    inv_image = np.fliplr(inv_image)
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(inv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def callback2(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    inv_image = np.array(cv_image)
    inv_image = np.flipud(inv_image)
    inv_image = np.fliplr(inv_image)
    try:
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(inv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
