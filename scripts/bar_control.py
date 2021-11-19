#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
stop_bar = False


def cb_bar(data):
	global stop_bar
	stop_bar = data.data


def pub_velocity(x, z, time):
	pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	vel = Twist()
	for i in range(0, int(time*10)):
		vel.linear.x = x
		vel.angular.z = z
		pub_vel.publish(vel)
		rospy.sleep(0.1)


def do_stop():
	pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
	flag_move_line = Bool()
	flag_move_line.data = False
	rospy.sleep(0.1)
	pub_line_move.publish(flag_move_line)
	
	# pub_velocity(0.2,0,1)
	for i in range(20,0, -2):
		pub_velocity(i/100,0,0.2)
	
	while stop_bar == True:
		pub_velocity(0,0,0.1)
	flag_move_line.data = True
	pub_line_move.publish(flag_move_line)


if __name__ == '__main__':
	rospy.init_node('stop_bar')
	sub_bar = rospy.Subscriber('bar', Bool, cb_bar, queue_size=1)
	while not rospy.is_shutdown():
		try:
			if(stop_bar == True):
				print("stop detected")
				do_stop()
				break
		except KeyboardInterrupt:
			break

