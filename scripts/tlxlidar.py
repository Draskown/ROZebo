#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import String
from nav_msgs.msg import Odometry
pub_image = rospy.Publisher('image_traffic_light', Image, queue_size=1)
pub_msg = rospy.Publisher('tl_msg', String, queue_size=1)
cvBridge = CvBridge()
counter = 1
msg = String()
enabled = True


def cb_odom(data):
	global enabled
	if data.pose.pose.position.x > 0 and data.pose.pose.position.y < 0:
		enabled = True
	else:
		enabled = False


def cbImageProjection(data):
	global counter
	
	if counter % 3 != 0:
		counter += 1
		return
	else: 
		couner = 1  
	
	cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
	cv_image_original = cv2.GaussianBlur(cv_image_original, (3, 3), 0)
	cv_image_gray = cv2.cvtColor(cv_image_original, cv2.COLOR_BGR2GRAY)
	
	circles = cv2.HoughCircles(cv_image_gray, cv2.HOUGH_GRADIENT, 1, 50, param2 = 20, minRadius = 5, maxRadius = 15 ) 
	
	if enabled and circles is not None:
		circles = np.round(circles[0,:]).astype("int")
		for x,y,r in circles:
			cv2.circle(cv_image_gray, (x,y),r,(0,255,0), 3)
			
		msg.data = "traffic light"
	else:
		msg.data = "none"
		
	pub_msg.publish(msg)
		
	temp = np.hsplit(cv_image_gray,2) 
	cv_image_gray = temp[1]
	pub_image.publish(cvBridge.cv2_to_imgmsg(cv_image_gray, "8UC1"))

	
if __name__ == '__main__':
	rospy.init_node('traffic_light_detector')
	sub_image = rospy.Subscriber('/camera/image', Image, cbImageProjection, queue_size=1)
	sub_odom = rospy.Subscriber('odom', Odometry, cb_odom, queue_size=1)
	while not rospy.is_shutdown():
			try:
				rospy.sleep(0.1)
			except KeyboardInterrupt:
				break
				cv2.destroyAllWindows()

