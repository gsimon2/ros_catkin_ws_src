#!/usr/bin/env python

### Code comes from:
# 	http://python.dronekit.io/examples/mission_basic.html
###
#from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import sys
import rospy
from pymavlink import mavutil
from dronekit_functions import get_location_metres, get_distance_metres, distance_to_current_waypoint, download_mission, adds_square_mission, is_vehicle_home
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import OverrideRCIn
import std_msgs.msg
import numpy as np
from obstacle_avoidance_functions import partition_vision, check_vision, parse_genome, findMiddle, ErleRover_Obstacle_Avoidance


def scan_callback(data):
	#partition data ranges into sections
	partitioned_vision = partition_vision(data, 5)
	
	# Use obstacle avoidance algorithm
	#nav_cmds = check_vision(data, partitioned_vision)
	nav_cmds= ErleRover_Obstacle_Avoidance(data)
	
	yaw = nav_cmds['yaw']
	throttle = nav_cmds['throttle']
	
	#print('Yaw: {} \t Throttle: {}'.format(yaw,throttle))

	msg = OverrideRCIn()
	msg.channels[0] = yaw
	msg.channels[1] = 0
	msg.channels[2] = throttle
	msg.channels[3] = 0
	msg.channels[4] = 0
	msg.channels[5] = 0
	msg.channels[6] = 0
	msg.channels[7] = 0
	
	obstacle_avoidance_cmds_pub.publish(msg)
	


def eval_start_callback(data):
	scan_sub = rospy.Subscriber("/scan", LaserScan,scan_callback)

	print("Starting evaluation")
	

	while rospy.get_param('simulation_running') is True:
		pass
		#if obstacle_avoidance_commands is not '':
		#	nav_cmds_pub.publish(obstacle_avoidance_commands)
	
	print('This sim is done!')
	scan_sub.unregister()
	
	msg = OverrideRCIn()
	msg.channels[2] = 1500
	obstacle_avoidance_cmds_pub.publish(msg)
	

### Set up ROS subscribers and publishers ###
rospy.init_node('rover_basic_obs_avoid_controller',anonymous=False)
#eval_start_sub = rospy.Subscriber('evaluation_start', std_msgs.msg.Empty, eval_start_callback)
obstacle_avoidance_cmds_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
scan_sub = rospy.Subscriber("/scan", LaserScan,scan_callback)

rospy.spin()
