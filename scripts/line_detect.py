#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Float64

pub_image = rospy.Publisher('image', Image, queue_size=1)
pub_error = rospy.Publisher('line_error', Float64, queue_size=1)
cvBridge = CvBridge()

def cbImageProjection(data):
	cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
	cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)
	yellow_detect, yellow_array = mask_yellow(cv_image_original)
	white_detect, white_array = mask_white(cv_image_original)
	detected = cv2.add(white_detect, yellow_detect)
	pub_image.publish(cvBridge.cv2_to_imgmsg(detected, "bgr8"))
	error = Float64()
	error.data = calculate_error(yellow_array, white_array)
	pub_error.publish(error)
    
def mask_yellow(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 27
	Hue_h = 41
	Saturation_l = 130
	Saturation_h = 255
	Lightness_l = 160
	Lightness_h = 255

	# define range of yellow color in HSV
	lower_yellow = np.array([Hue_l, Saturation_l, 	Lightness_l])
	upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(img, img, mask = mask)
	fraction_num = np.count_nonzero(mask)
	point_arr = []
	stop_flag = False
	if fraction_num > 50:
		k = 0
		jold = 0
		for i in range(mask.shape[0]-1,0,-15):
			if stop_flag == True:
				break
			for j in range(0,mask.shape[1],15):
				if mask[i,j] > 0:
					point_arr.append([j,i])
					k+=1
					if abs(j-jold) > 80 and k > 1:
						point_arr.pop()
						stop_flag = True
				jold = j
				break

		if(len(point_arr) > 0):
			point_before = point_arr[0]
			for point in point_arr:
				res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
				point_before = point
	return res, point_arr
   

def mask_white(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 0
	Hue_h = 25
	Saturation_l = 0
	Saturation_h = 36
	Lightness_l = 180
	Lightness_h = 255

	# define range of yellow color in HSV
	lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_white, upper_white)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(img, img, mask = mask)
	fraction_num = np.count_nonzero(mask)
	point_arr = []
	stop_flag = False
	if fraction_num > 50:
		k = 0
		jold = 0
		for i in range(mask.shape[0]-1,0,-20):
			if stop_flag == True:
				break
			for j in range(mask.shape[1]-1,0,-20):
				if mask[i,j] > 0:
					point_arr.append([j,i])
					k+=1
					if abs(j-jold) > 80 and k > 1:
						point_arr.pop()
						stop_flag = True
					jold = j
					break

		if len(point_arr) > 0:
			point_before = point_arr[0]
			for point in point_arr:
				res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
				point_before = point
			
	return res, point_arr
  
  
def calculate_error(yellow_array, white_array):
	error_yell = 0
	error_white = 0
	weight = 0
	i = 1

	for yel in yellow_array:
	#when yel[2] = 600 then weight = 0 and if yel[2] = 0 wheight = 1
		weight = yel[1]*0.0017 + 1
		error_yell = weight*(30 - yel[0]) + error_yell
		i+=1
	error_yell = error_yell/i
	for white in white_array:
		weight = white[1]*0.0017 + 1
		error_white = weight*(300 - white[0]) + error_white
		i+=1
	error_white = error_white/i
	print("white "+ str(error_white) + " yellow "+ str(error_yell))
	if error_white < 30:
		return error_yell
	elif error_yell < 30:
		return error_white
	else:
		return (error_white + error_yell)/2
		
if __name__ == '__main__':
	rospy.init_node('image_projection')
	sub_image = rospy.Subscriber('/camera_line', Image, cbImageProjection, queue_size = 1)
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

