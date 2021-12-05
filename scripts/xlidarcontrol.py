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
		tl_sub = rospy.Subscriber('image_traffic_light', Image, self.cb_tl_img, queue_size = 1)
		bar_sub = rospy.Subscriber('image_bar', Image, self.cb_bar_img, queue_size=1)
		camera_sub = rospy.Subscriber('camera/image', Image, self.cb_cam, queue_size=1)
		scan_sub = rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
		
		self.img_pub = rospy.Publisher('decided_img', Image, queue_size=1)
		self.scan_pub = rospy.Publisher('scan_img', Image, queue_size=1)
		
		self.mode = 0
		self.cvBridge = CvBridge()
		self.overallscans = 0
		
		self.empty_image = np.zeros(shape=(400, 400), dtype=np.uint8)
		
		
	def cb_sign(self, sign_name):
		pass


	def cb_cam(self, image):
		if self.mode == 0:
			self.img_pub.publish(image)


	def cb_tl_img(self, image):
		if self.mode == 1:
			self.img_pub.publish(image)


	def cb_bar_img(self, image):
		if self.mode == 2:
			self.img_pub.publish(image)


	def cb_sign_img(self, image):
		if self.mode == 3:
			self.img_pub.publish(image)
	
	
	def cb_scan(self, data):		
		closest = 2
		for i in range (359):
			if i%10 != 0:
				if data.ranges[i] < closest:
					closest = data.ranges[i]
			else:
				if closest != 2:
					x = int(closest*np.cos((i + 180)*np.pi/180)*100 + 200)
					y = int(closest*np.sin((i + 180)*np.pi/180)*100 + 200)
				
					self.empty_image[x, y] = 255
					closest = 2
		
		self.overallscans += 1
		
		#if self.overallscans == 20:
		self.overallscans = 0
		
		self.scan_pub.publish(self.cvBridge.cv2_to_imgmsg(self.empty_image, "mono8"))
		self.empty_image = np.zeros(shape=(400, 400), dtype=np.uint8)
	

	def main(self):
		key = self.getKey()
		
		if key == "\x03":
			exit(0)
		elif key == "0":
			self.mode = 0
		elif key == "1":
			self.mode = 1
		elif key == "2":
			self.mode = 2
		elif key == "3":
			self.mode = 3


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
	

if __name__ == '__main__':
	node = Xlidarcontrol()
	while not rospy.is_shutdown():
		try:
			node.main()
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

