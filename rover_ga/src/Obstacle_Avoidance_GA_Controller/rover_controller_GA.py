#!/usr/bin/env python
#
# Rover Controller
#
# Top level movement controller for the rover
#	Spawns ROS nodes for different functionalities, listens to their
#	outputs and relays navigation commands to MAVROS
#
#	Current Functions:
#		-obstacle avoidance node 
#
# GAS 2017-07-24

import os
import argparse
import json
import zmq
import rospy
import time
import subprocess
import socket
import datetime
import random

import std_msgs.msg

from argparse import RawTextHelpFormatter
from mavros_msgs.msg import OverrideRCIn

obstacle_avoidance_commands = ''

### Start lower level controllers ###
###		Starts lower level controller ROS nodes that make up this controller
###		lower level controllers:
###			-Obstacle avoidance
###			-Map navigation (in testing)
def start_lower_level_controllers():
	os.system("rosrun rover_ga obstacle_avoidance_GA.py")
	os.system("rosrun rover_ga map_navigation_GA.py")

### Shutdown Hook ###
### 	This is the last function executed before the script exits 
###		Cleans up all lower-level controllers that were spawned
def shutdown_hook():
	cmd_str = "pkill -f obstacle_avoidance.py"
	os.system(cmd_str)
	cmd_str = "pkill -f map_navigation.py"
	os.system(cmd_str)

def obstacle_avoidance_callback(data):
	global obstacle_avoidance_commands
	obstacle_avoidance_commands = data
	nav_cmds_pub.publish(data)

def eval_start_callback(data):
	global obstacle_avoidance_commands
	obstacle_avoidance_commands = ''
	obstacle_avoidance_cmds_sub = rospy.Subscriber('obstacle_avoindance_cmds', OverrideRCIn, obstacle_avoidance_callback)
	
	print("Starting evaluation")
	

	while rospy.get_param('simulation_running') is True:
		pass
		#if obstacle_avoidance_commands is not '':
		#	nav_cmds_pub.publish(obstacle_avoidance_commands)
	
	print('This sim is done!')
	obstacle_avoidance_cmds_sub.unregister()
	#Stop the rover
	msg = OverrideRCIn()
	msg.channels[2] = 1500
	nav_cmds_pub.publish(msg)
	

### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""
Top level movement controller for the rover
	Spawns ROS nodes for different functionalities, listens to their
	outputs and relays navigation commands to MAVROS
	""", formatter_class=RawTextHelpFormatter)

args= parser.parse_args()

### Set up ROS subscribers and publishers ###
rospy.init_node('rover_controller',anonymous=False)
eval_start_sub = rospy.Subscriber('evaluation_start', std_msgs.msg.Empty, eval_start_callback)
nav_cmds_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)


### Start lower level controllers ###
start_lower_level_controllers()


print("Done!")
rospy.on_shutdown(shutdown_hook)
rospy.spin()
