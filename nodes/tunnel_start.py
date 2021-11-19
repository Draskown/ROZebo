#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
ranges = list()
tunnel = False
in_tunnel = False


def cb_sign(data):
	global tunnel
	if(data.data == "tunnel"):
		tunnel = True


def cb_scan(data):
	global ranges
	ranges = data.ranges


def pub_velocity(x, z, time):
	pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	vel = Twist()
	for i in range(0, int(time*10)):
		vel.linear.x = x
		vel.angular.z = z
		pub_vel.publish(vel)
		rospy.sleep(0.1)


def do_tunnel():
	pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
	flag_move_line = Bool()
	flag_move_line.data = False
	rospy.sleep(3)
	rospy.sleep(0.1)
	pub_line_move.publish(flag_move_line)
	pub_velocity(0.2,0,2)
	for i in range(20,0, -2):
		pub_velocity(i/100,0,0.3)


def sign(data):
	if(data >= 0):
		return 1
	else:
		return -1


def in_tunnel_go():
	global ranges
	vel_x = 0.12
	vel_z = 0
	error = 4*(0.23 - ranges[300])
	if(abs(error) > 1.5):
		error = sign(error)*1.5
	vel_z = error
	for i in range(0,360,1):
		if(ranges[i] < 0.15):
			if(i <= 30 or i >= 330):
				vel_x = -0.09
				vel_z = 0.4    
	pub_velocity(vel_x, vel_z, 0.1)


if __name__ == '__main__':
	rospy.init_node('tunnel')
	sub_sign = rospy.Subscriber('sign', String, cb_sign, queue_size=1)
	sub_bar = rospy.Subscriber('scan', LaserScan, cb_scan, queue_size=1)
	while not rospy.is_shutdown():
		try:
			if(tunnel == True and in_tunnel == False):
				print("tunnel detected")
				do_tunnel()
				in_tunnel = True
			elif(in_tunnel == True):
				in_tunnel_go()
		except KeyboardInterrupt:
			break

