#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import  String, Bool
from geometry_msgs.msg import Twist
light = False
enabled = False


def cb_ts(data):
	global enabled
	
	if int(data.data) == 1 or data.data == "2":
		enabled = True


def cb_traffic_light(data):
	global light
	if enabled:
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
	
	for i in range(5,0, -1):
		pub_velocity(i/100,0,0.2)
	
	print("published stop msg")
	while( light == True):
		pub_velocity(0, 0, 0.1)
	flag_move_line.data = True
	pub_line_move.publish(flag_move_line)


if __name__ == '__main__':
	rospy.init_node('light_controller')
	sub_sign = rospy.Subscriber('traffic_light', String, cb_traffic_light, queue_size=1)
	sub_state = rospy.Subscriber('state', String, cb_ts, queue_size=1)
	while not rospy.is_shutdown():
		try:
			if(light == True):
				print("start traffic light mission")
				do_traffic_light()
				break
		except KeyboardInterrupt:
			break

