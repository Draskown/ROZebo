#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
distance = 0
parking = False


def cb_sign(data):
	global parking
	if(data.data == "parking"):
		parking = True


def cb_scan(data):
	global distance
	counter = 0
	for i in range(200,300):
		if(data.ranges[i] < 1):
			distance = distance + data.ranges[i]
			counter = counter + 1
	if(counter != 0):
		distance = distance/counter
	else:
		distance = 0


def pub_velocity(x, z, time):
	pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	vel = Twist()
	for i in range(0, int(time*10)):
		vel.linear.x = x
		vel.angular.z = z
		pub_vel.publish(vel)
		rospy.sleep(0.1)


def do_parking():
	pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
	flag_move_line = Bool()
	flag_move_line.data = False
	rospy.sleep(0.1)
	pub_line_move.publish(flag_move_line)    
	
	#print("published stop msg")
	pub_velocity(0, 0, 0.5)
	
	#print("published vel")
	pub_velocity(0, -0.4,4)
	pub_velocity(0, 0, 0.3)
	pub_velocity(0.13, 0,2)
	pub_velocity(0, 0, 0.3)
	pub_velocity(0, 0.4,4)
	pub_velocity(0, 0, 0.5)
	pub_velocity(0, -0.4,4)
	pub_velocity(0, 0, 0.3)
	pub_velocity(-0.13, 0,2)
	pub_velocity(0, 0, 0.3)
	pub_velocity(0, 0.4,4)
	pub_velocity(0,0,0.5)
	flag_move_line.data = True
	pub_line_move.publish(flag_move_line)


if __name__ == '__main__':
	rospy.init_node('parking')
	sub_sign = rospy.Subscriber('sign', String, cb_sign, queue_size=1)
	sub_scan = rospy.Subscriber('scan', LaserScan, cb_scan, queue_size=1)
	while not rospy.is_shutdown():
		try:
			if(parking == True):
				print("start parking mission")
				rospy.sleep(6)
				print(distance)
				if(distance > 1 or distance == 0):
					do_parking()
				else:
					print("place is occupied")
					rospy.sleep(2)
					do_parking()
				break
		except KeyboardInterrupt:
			break

