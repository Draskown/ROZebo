#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import String, Bool
pub_image = rospy.Publisher('image_traffic_light', Image, queue_size=1)
pub_traffic_light = rospy.Publisher('traffic_light', String, queue_size=1)
cvBridge = CvBridge()
counter = 1
light_msg = String()


plan = True


def cbPlan(data):
	global plan
	plan = data.data


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
	green_x, green_y  = mask_green(cv_image_original)
	yellow_x, yellow_y = mask_yellow(cv_image_original)
	red_x, red_y  = mask_red(cv_image_original)
	
	global light_msg
	light_msg.data = "none"
	
	if circles is not None:
		circles = np.round(circles[0,:]).astype("int")
		for x,y,r in circles:
			cv2.circle(cv_image_gray, (x,y),r,(0,255,0), 7)
			green_err = calc_err(green_x, green_y, x, y) 
			red_err = calc_err(red_x, red_y, x, y)
			yellow_err =  calc_err(yellow_x, yellow_y, x, y)
			
			if green_err < 400:
				light_msg.data = "green"
			if red_err < 400:
				light_msg.data = "red"
			if yellow_err < 400:
				light_msg.data = "yellow"
	
	pub_traffic_light.publish(light_msg)
	temp = np.hsplit(cv_image_gray,2) 
	cv_image_gray = temp[1]
	pub_image.publish(cvBridge.cv2_to_imgmsg(cv_image_gray, "8UC1"))


def calc_err(color_x, color_y, circle_x, circle_y):
	if color_x*color_y*circle_y*circle_x > 0:
		x_err = (color_x - circle_x)**2
		y_err = (color_y - circle_y)**2
		err = x_err+y_err / 2
		return err
	return 100000


def mask_yellow(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 20
	Hue_h = 35
	Saturation_l = 100
	Saturation_h = 255
	Lightness_l = 50
	Lightness_h = 255
  
  # define range of yellow color in HSV
	lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])
  
  # Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	temp = np.hsplit(mask,2)
	mask = temp[1]
	fraction_num = np.count_nonzero(mask)
	if fraction_num > 100:
		for y in range(0, mask.shape[0]-1,5):
			for x in range(0,mask.shape[1]-1,5):
				if mask[y,x]>0:
					return(x+160,y) 
	return 0, 0


def mask_red(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 0
	Hue_h = 10
	Saturation_l = 30
	Saturation_h = 255
	Lightness_l = 48
	Lightness_h = 255
	
	# define range of yellow color in HSV
	lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_red = np.array([Hue_h, Saturation_h, Lightness_h])
	
	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_red, upper_red)
	temp = np.hsplit(mask,2) 
	mask = temp[1]
	fraction_num = np.count_nonzero(mask)
	if fraction_num > 100:
		for y in range(0, mask.shape[0]-1,5):
			for x in range(0,mask.shape[1]-1,5):
				if mask[y,x]>0:
					return(x+160,y)
	return 0, 0


def mask_green(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 46
	Hue_h = 76
	Saturation_l = 86
	Saturation_h = 255
	Lightness_l = 50
	Lightness_h = 255
	
	# define range of yellow color in HSV
	lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_green = np.array([Hue_h, Saturation_h, Lightness_h])
	
	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_green, upper_green)
	temp = np.hsplit(mask,2)
	mask = temp[1]
	fraction_num = np.count_nonzero(mask)
	if fraction_num > 100:
		for y in range(0, mask.shape[0]-1,5):
			for x in range(0,mask.shape[1]-1,5):
				if mask[y,x]>0:
					return(x+160,y) 
	return 0, 0
	
	
if __name__ == '__main__':
	rospy.init_node('traffic_light_detector')
	sub_image = rospy.Subscriber('/camera/image', Image, cbImageProjection, queue_size=1)
	sub_plan = rospy.Subscriber('plan', Bool, cbPlan, queue_size = 1)
	while not rospy.is_shutdown():
			try:
				if light_msg.data != "green" and plan:
					rospy.sleep(0.1)
				else:
					break
					cv2.destroyAllWindows()
			except KeyboardInterrupt:
				break
				cv2.destroyAllWindows()

