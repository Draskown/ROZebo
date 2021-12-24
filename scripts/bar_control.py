#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
stop_bar = False
crunches = False
orientation = 0
pose_x = 0
pose_y = 0
error = 0
target_orientation = 0.7
target_position = {'x': -1.828144623628639, 'y': 1.352652814982841}



def cb_odom(msg):
	global crunches, orientation, error, pose_x, pose_y
	pose_x = msg.pose.pose.position.x
	pose_y = msg.pose.pose.position.y
	orientation = msg.pose.pose.orientation.z
	
	if pose_x > -1.92 and pose_x < -1.72 and pose_y > 0 and pose_y < 1.7:
		crunches = True
	

def cb_bar(data):
	global stop_bar
	stop_bar = data.data


def pubvel(x, z, time):
	pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	velocity = Twist()
	velocity.linear.x = x
	velocity.angular.z = z
	pub_vel.publish(velocity)
	rospy.sleep(time)	


def do_stop():
	global error
	
	rospy.sleep(1.45)	
	
	pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
	#pub_tunnel = rospy.Publisher('tunnel', Bool, queue_size=1)
	flag_move_line = Bool()
	flag_move_line.data = False
	rospy.sleep(0.1)
	pub_line_move.publish(flag_move_line)
	
	print('moving forward')
	
	while not (abs(pose_x + 1.75) < 0.15 and pose_y < 1):
		error = target_orientation - abs(orientation)
		pubvel(0.2, -error*5, 0.1)
	
	for i in range(20,0, -2):
		pubvel(i/100,0,0.2)
		
	print('waiting for the bar to open')
	while stop_bar:
		pubvel(0.0, 0.0, 0.1)
	
	while pose_y > 0:
		error = target_orientation - abs(orientation)
		pubvel(0.2, -error*5, 0.1)
	

if __name__ == '__main__':
	rospy.init_node('bar_control')
	sub_bar = rospy.Subscriber('bar', Bool, cb_bar, queue_size=1)
	sub_odom = rospy.Subscriber('odom', Odometry, cb_odom, queue_size=1)
	
	while not rospy.is_shutdown():
		try:
			if crunches:
				do_stop()
				break
		except KeyboardInterrupt:
			break

