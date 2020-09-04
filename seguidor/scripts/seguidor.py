#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def image_callback(ros_image):
  print 'got an image'
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  (rows,cols,channels) = cv_image.shape
  #if cols > 200 and rows > 200 :
  #    cv2.circle(cv_image, (100,100),90, 255)
  #font = cv2.FONT_HERSHEY_SIMPLEX
  #cv2.putText(cv_image,'P3DX - Seguidor de Linha!',(10,350), font, 1,(30,255,255),2,cv2.LINE_AA)
  cv2.imshow("Camera Frontal do P3DX", cv_image)
  cv2.waitKey(3)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("/p3dx/front_camera/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


