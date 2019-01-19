#!/usr/bin/env python
from __future__ import print_function


import sys
import cv2

import imutils
import math

import rospy
import roslib
roslib.load_manifest('uoft_hacks')
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


pixel_target = (320, 240)


class ImageConverter:
       
    def __init__(self):
        self.image_pub = rospy.Publisher("detected_target",Image,queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imageCallback)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect",Image,self.depthCallback)
        self.depth_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camInfoCallback)

        self.recent_depth_image = None
        self.initialized = False

        self.cx = None
        self.cy = None
        self.inv_fx = None
        self.inv_fy = None
        
        
    def imageCallback(self,data):
        # No depth image yet
        if not self.initialized:
            return

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

                # #compute distance to centroid
                # z_metric = float(self.recent_depth_image[int(y), int(x)]) * 0.00001
                # pt_x = z_metric * ((x - self.cx) * self.inv_fx)
                # pt_y = z_metric * ((y - self.cy) * self.inv_fy)
                # pt_z = z_metric

                dist_to_target = self.recent_depth_image[int(y), int(x)]
                print("Depth image reading ", dist_to_target)
                print("Centroid of Object at: ", int(y), int(x))

                x_error = pixel_target[0] - int(x)
                dist_error = 1.0 - dist_to_target
                self.pubCmdVel(x_error, dist_error)


        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    def pubCmdVel(self, x_error, dist_error):
        msg = Twist()
        if (x_error > 30):
            msg.angular.z = 30*0.0174533
        elif (x_error < -30):
            msg.angular.z = -30*0.0174533
        else: 
            msg.angular.z = float(x_error) * 0.0174533

        msg.linear.x = float(dist_error) * -0.5
        if msg.linear.x > 0.6:
            msg.linear.x = 0.6
        elif msg.linear.x < -0.6:
            msg.linear.x = -0.6

        self.cmd_vel_pub.publish(msg)
  
    def depthCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.recent_depth_image = cv_image
        self.initialized = True

    def camInfoCallback(self, data):
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.inv_fx = data.K[0]
        self.inv_fy = data.K[4]

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