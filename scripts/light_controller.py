#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import  String, Bool
from geometry_msgs.msg import Twist
light = False


def cb_traffic_light(data):
	global light
	if(data.data == "yellow" or data.data == "red"):
		light = True
	elif(data.data == "green"):
		light = False


def pub_velocity(x, z, time):
	pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	vel = Twist()
	for i in range(0, int(time*10)):
		vel.linear.x = x
		vel.angular.z = z
		pub_vel.publish(vel)
		rospy.sleep(0.1)


def do_traffic_light():
	global light
	
	pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
	flag_move_line = Bool()
	flag_move_line.data = False
	rospy.sleep(0.1)
	pub_line_move.publish(flag_move_line)
	
	print("published stop msg")
	while( light == True):
		pub_velocity(0, 0, 0.1)
	flag_move_line.data = True
	pub_line_move.publish(flag_move_line)


if __name__ == '__main__':
	rospy.init_node('traffic_light_controller')
	sub_sign = rospy.Subscriber('traffic_light', String, cb_traffic_light, queue_size=1)
	while not rospy.is_shutdown():
		try:
			if(light == True):
				print("start traffic light mission")
				do_traffic_light()
				break
		except KeyboardInterrupt:
			break

