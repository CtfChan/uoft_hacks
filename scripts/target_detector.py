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

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

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

        self.initializeTracker()
        self.tracker_initialized = False


    def initializeTracker(self):
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
        self.tracker_type = tracker_types[2]
    
        if int(minor_ver) < 3:
            self.tracker = cv2.Tracker_create(self.tracker_type)
        else:
            if self.tracker_type == 'BOOSTING':
                self.tracker = cv2.TrackerBoosting_create()
            if self.tracker_type == 'MIL':
                self.tracker = cv2.TrackerMIL_create()
            if self.tracker_type == 'KCF':
                self.tracker = cv2.TrackerKCF_create()
            if self.tracker_type == 'TLD':
                self.tracker = cv2.TrackerTLD_create()
            if self.tracker_type == 'MEDIANFLOW':
                self.tracker = cv2.TrackerMedianFlow_create()
            if self.tracker_type == 'GOTURN':
                self.tracker = cv2.TrackerGOTURN_create()
            if self.tracker_type == 'MOSSE':
                self.tracker = cv2.TrackerMOSSE_create()
            if self.tracker_type == "CSRT":
                self.tracker = cv2.TrackerCSRT_create()
    
    def imageCallback(self,data):
        # No depth image yet
        if not self.initialized:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = cv_image
        if self.tracker_initialized:

            # Update tracker
            ok, bbox = self.tracker.update(frame)

            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                self.tracker_initialized = False

            # Display tracker type on frame
            cv2.putText(frame, self.tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

            c_x = int(bbox[0]) + int(bbox[2] / 2.0) 
            c_y = int(bbox[1]) + int(bbox[3] / 2.0)


            sum_dist_to_target = 0.0
            num_of_non_nan_elements = 0.0
            for x in range(c_x - 5, c_x + 5 + 1 ):
                for y in range(c_y - 5, c_y + 5 + 1 ):
                    if y >= 0 and y < frame.shape[0] and x >= 0 and x < frame.shape[1]:
                        if self.recent_depth_image[y,x]:
                            sum_dist_to_target += float(self.recent_depth_image[y,x])
                            num_of_non_nan_elements += 1.0
            
            if num_of_non_nan_elements != 0:
                dist_to_target = sum_dist_to_target / float(num_of_non_nan_elements)
                print("Depth image reading ", dist_to_target)
                print("Centroid of Object at: ", int(c_y), int(c_x))

                x_error = pixel_target[0] - int(c_x)
                dist_error = 1.0 - dist_to_target
                self.pubCmdVel(x_error, dist_error)

        else:

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
                x,y,w,h = cv2.boundingRect(c)
                center = (x + w/2, y + h/2)

                # only proceed if the radius meets a minimum size
                if w*h > 100:
                    bbox = (x,y,w,h)
                    # Initialize tracker with first frame and bounding box
                    self.initializeTracker()
                    ok = self.tracker.init(frame, bbox)
                    self.tracker_initialized = True

        cv2.imshow("Image window", frame)
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