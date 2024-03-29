#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Bool, String
pub_image = rospy.Publisher('image_bar', Image, queue_size=1)
pub_bar = rospy.Publisher('bar', String, queue_size=1)
cvBridge = CvBridge()
counter = 1
enable = False

plan = True


def cbPlan(data):
	global plan
	plan = data.data


def cb_ts(data):
	if int(data.data) >= 5:
		global enable
		enable = True


def cbImageProjection(data):
	
	if not enable:
		return
	
	global counter
	if counter % 3 != 0:
		counter += 1
		return
	else:
		counter = 1

	cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
	cv_image_original = cv2.GaussianBlur(cv_image_original, (3, 3), 0)
	bar_msg = String()
	temp, res = mask_red(cv_image_original)
	
	if temp:
		bar_msg.data = "bar"
	else:
		bar_msg.data = "none"
	
	pub_bar.publish(bar_msg)
	pub_image.publish(cvBridge.cv2_to_imgmsg(res, "8UC1"))


def mask_red(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 0
	Hue_h = 10
	Saturation_l = 30
	Saturation_h = 255
	Lightness_l = 48
	Lightness_h = 255
	
	# define range of red color in HSV
	lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_red = np.array([Hue_h, Saturation_h, Lightness_h])
	
	# Threshold the HSV image to get only red colors
	mask = cv2.inRange(hsv, lower_red, upper_red)
	mask = cv2.erode(mask, (4,4), iterations = 6) #for detection of big rectangle
	fraction_num = np.count_nonzero(mask)
	
	if fraction_num > 150:
		return True, mask
	else:
		return False, mask


if __name__ == '__main__':
	rospy.init_node('bar_detect')
	sub_image = rospy.Subscriber('/camera/image', Image, cbImageProjection, queue_size=1)
	sub_tl = rospy.Subscriber('state', String, cb_ts, queue_size=1)
	sub_plan = rospy.Subscriber('plan', Bool, cbPlan, queue_size = 1)
	while not rospy.is_shutdown():
		try:
			if plan:
				rospy.sleep(0.1)
			else:
				break
				cv2.destroyAllWindows()
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

