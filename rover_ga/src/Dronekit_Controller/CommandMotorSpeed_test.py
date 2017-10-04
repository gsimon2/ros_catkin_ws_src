#!/usr/bin/env python
import time
import math
import sys
import rospy


import mav_msgs.msg as mav_msgs
import collections
history_queue = collections.deque(maxlen=200) 

def echo_cb(data):
	global history_queue
	yaw = data.motor_speed[0] + 1000
	history_queue.append(yaw)
	print(sum(history_queue)/len(history_queue))
	
	
	

### Set up ROS subscribers and publishers ###
rospy.init_node('Command_Motor_Speed',anonymous=False)
test_sub = rospy.Subscriber('/rover/command/motor_speed',  mav_msgs.CommandMotorSpeed, echo_cb)

rospy.spin()

