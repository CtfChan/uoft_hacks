#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('uoft_hacks')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import imutils



class ImageConverter:

  def __init__(self):
    self.image_pub = rospy.Publisher("detected_target",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imageCallback)
    self.depth_sub = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw",Image,self.depthCallback)

  def depthCallback(self,data):
    pass  

  def imageCallback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    green_lower = (20, 100, 100)
    green_upper = (64, 255, 255)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, green_lower, green_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    
    if len(cnts) > 0:
      # find the largest contour in the mask, then use
      # it to compute the minimum enclosing circle and
      # centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
  
      # only proceed if the radius meets a minimum size
      if radius > 10:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(cv_image, (int(x), int(y)), int(radius),
          (0, 255, 255), 2)
        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
      

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = ImageConverter()
  rospy.init_node('target_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)