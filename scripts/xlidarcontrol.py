#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, os, select, sys, tty, termios
import cv2, numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


class Xlidarcontrol():
	def __init__(self):
		
		if os.name != 'nt':
			self.settings = termios.tcgetattr(sys.stdin)
		
		rospy.init_node('control', disable_signals = True)
		
		sign_msg_sub = rospy.Subscriber('sign', String, self.cb_sign, queue_size=1)
		sign_img_sub = rospy.Subscriber('sign_image', Image, self.cb_sign_img, queue_size=1)
		tl_msg_sub = rospy.Subscriber('tl_msg', String, self.cb_tl, queue_size=1)
		tl_sub = rospy.Subscriber('image_traffic_light', Image, self.cb_tl_img, queue_size = 1)
		bar_sub = rospy.Subscriber('image_bar', Image, self.cb_bar_img, queue_size=1)
		bar_msg_sub = rospy.Subscriber('bar', String, self.cb_bar, queue_size=1)
		camera_sub = rospy.Subscriber('camera/image', Image, self.cb_cam, queue_size=1)
		scan_sub = rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
		
		self.img_pub = rospy.Publisher('decided_img', Image, queue_size=1)
		self.scan_pub = rospy.Publisher('scan_img', Image, queue_size=1)
		
		self.mode = 1
		self.angle = 0
		self.closest = 0.75
		self.cvBridge = CvBridge()
		self.sign_text = "none"
		self.bar_text = "none"
		self.tl_text = "none"
		
		self.empty_image = np.zeros(shape=(400, 400), dtype=np.uint8)

		
	def cb_sign(self, sign_name):
		self.sign_text = sign_name.data


	def cb_tl(self, data):
		self.tl_text = data.data
		
	
	def cb_bar(self, data):
		self.bar_text = data.data
	
		
	def cb_cam(self, image):
		try:
			if self.mode == 1:
				self.pub_image(image, "rgb8")
		except AttributeError:
			return


	def cb_tl_img(self, image):
		try:
			if self.mode == 2:
				self.pub_image(image, "8UC1")
		except AttributeError:
			return
		

	def cb_bar_img(self, image):
		try:
			if self.mode == 3:
				self.pub_image(image, "8UC1")
		except AttributeError:
			return


	def cb_sign_img(self, image):
		try:
			if self.mode == 4:
				self.pub_image(image, "rgb8")
		except AttributeError:
			return
	
	
	def cb_scan(self, data):		
		closest_ten = 2
		self.closest = 0.75
		for i in range (359):
			if data.ranges[i] < self.closest and (i > 270 or i < 90):
				self.closest = data.ranges[i]
				self.angle = i
			elif self.sign_text == "tunnel" and data.ranges[i] < self.closest and (i > 350 or i < 90):
				self.closest = data.ranges[i]
				if i > 180:
					self.angle = i-360
				else:
					self.angle = i+1
			
			if i%10 != 0:
				if data.ranges[i] < closest_ten:
					closest_ten = data.ranges[i]
			else:
				if closest_ten != 2:
					x = int(closest_ten*np.cos((i + 180)*np.pi/180)*100 + 200)
					y = int(closest_ten*np.sin((i + 180)*np.pi/180)*100 + 200)
				
					self.empty_image[x, y] = 255
					closest_ten = 2
		
		self.scan_pub.publish(self.cvBridge.cv2_to_imgmsg(self.empty_image, "8UC1"))
		self.empty_image = np.zeros(shape=(400, 400), dtype=np.uint8)
	
	
	def pub_image(self, img, encoding):		
		seen_object = "none"
		if self.sign_text != "none" and self.bar_text == "none" and self.tl_text == "none":
			seen_object = self.sign_text
		elif self.sign_text == "none" and self.bar_text != "none" and self.tl_text == "none":
			seen_object = self.bar_text
		elif self.sign_text == "none" and self.bar_text == "none" and self.tl_text != "none":
			seen_object = self.tl_text
		elif self.sign_text != "none" and self.bar_text != "none" and self.tl_text == "none":
			seen_object = self.bar_text
		elif self.sign_text != "none" and self.bar_text == "none" and self.tl_text != "none":
			seen_object = self.tl_text
		elif self.sign_text == "none" and self.bar_text != "none" and self.tl_text == "none":
			seen_object = self.bar_text
		elif self.sign_text != "none" and self.bar_text != "none" and self.tl_text != "none":
			seen_object = self.bar_text
			
		cv_image = self.cvBridge.imgmsg_to_cv2(img)
		cv_image = cv2.putText(cv_image, seen_object,
														(3, 18),
														cv2.FONT_HERSHEY_SIMPLEX,
														0.75,
														(255, 0, 155),
														1,
														2)
		
		if seen_object != "none" and seen_object != "traffic light":
			
			text = "distance: {} cm".format(int(self.closest*125))
			
			cv_image = cv2.putText(cv_image, text,
															(3, 34),
															cv2.FONT_HERSHEY_SIMPLEX,
															0.4,
															(255, 0, 155),
															1,
															2)
															
			text = "angle: {}".format(self.angle)
			
			cv_image = cv2.putText(cv_image, text,
															(3, 44),
															cv2.FONT_HERSHEY_SIMPLEX,
															0.4,
															(255, 0, 155),
															1,
															2)
						
		self.img_pub.publish(self.cvBridge.cv2_to_imgmsg(cv_image, encoding))
	
	
	def getKey(self):
		if os.name == 'nt':
			if sys.version_info[0] >= 3:
				return msvcrt.getch().decode()
			else:
				return msvcrt.getch()

		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist:
			key = sys.stdin.read(1)
		else:
			key = ''

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key
	
	
	def main(self):
		key = self.getKey()
		
		if key == "\x03":
			exit(0)
		elif key == "1":
			self.mode = 1
		elif key == "2":
			self.mode = 2
		elif key == "3":
			self.mode = 3
		elif key == "4":
			self.mode = 4


if __name__ == '__main__':
	node = Xlidarcontrol()
	while not rospy.is_shutdown():
		try:
			node.main()
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

