#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import String
pub_image = rospy.Publisher('image_traffic_light', Image, queue_size=1)
pub_msg = rospy.Publisher('tl_msg', String, queue_size=1)
cvBridge = CvBridge()
counter = 1
circlesbool = False


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
	
	global circlesbool
	if circles is not None:
		circlesbool = True
		circles = np.round(circles[0,:]).astype("int")
		for x,y,r in circles:
			cv2.circle(cv_image_gray, (x,y),r,(0,255,0), 3)
	else:
		circlesbool = False
	
	temp = np.hsplit(cv_image_gray,2) 
	cv_image_gray = temp[1]
	pub_image.publish(cvBridge.cv2_to_imgmsg(cv_image_gray, "8UC1"))

	
if __name__ == '__main__':
	rospy.init_node('traffic_light_detector')
	sub_image = rospy.Subscriber('/camera/image', Image, cbImageProjection, queue_size=1)
	while not rospy.is_shutdown():
			try:
				rospy.sleep(0.1)
			except KeyboardInterrupt:
				break
				cv2.destroyAllWindows()

