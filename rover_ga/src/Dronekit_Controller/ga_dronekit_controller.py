#!/usr/bin/env python

### Code comes from:
# 	http://python.dronekit.io/examples/mission_basic.html
###
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
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

last_vehicle_mode = VehicleMode("AUTO")

connection_string = '127.0.0.1:14550'

def scan_callback(data):
	global vehicle
	global last_vehicle_mode
	
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
	
	
	engage_obstacle_avoidance_distance = 5
	
	#print(partitioned_vision)
	
	try:
		# Check to see if an object is in the path of the rover
		if all(i >= engage_obstacle_avoidance_distance for i in partitioned_vision):
		#if findMiddle(partitioned_vision) >= engage_obstacle_avoidance_distance:
			# All is good
			if(vehicle.mode == VehicleMode("MANUAL")):
				#print('Clear of obstacle! Switching to {}'.format(last_vehicle_mode))
				vehicle.mode = last_vehicle_mode
		else:
			if(vehicle.mode != VehicleMode("MANUAL")):
				print('Object detected! Obstacle avoidance engaged!')
				last_vehicle_mode = vehicle.mode
			vehicle.mode = VehicleMode("MANUAL")
			obstacle_avoidance_cmds_pub.publish(msg)
	except:
		pass



def eval_start_callback(data):
	global vehicle
	scan_sub = rospy.Subscriber("/scan", LaserScan,scan_callback)
	rospy.set_param('mission_success', False)
	
	print("Starting evaluation")
	
	# Connect to the Vehicle
	print 'Connecting to vehicle on: %s' % connection_string
	vehicle = connect(connection_string, wait_ready=True)
	
	# Get Vehicle Home location - will be `None` until first set by autopilot
	while not vehicle.home_location:
	    cmds = vehicle.commands
	    cmds.download()
	    cmds.wait_ready()
	    if not vehicle.home_location:
	        print " Waiting for home location ..."
	        vehicle.home_location = vehicle.location.global_frame
	# We have a home location, so print it!        
	print "\n Home location: %s" % vehicle.home_location
	
	
	print 'Create a new mission (for current location)'
	adds_square_mission(vehicle, vehicle.location.global_frame,5)
	time.sleep(1)
	
	print "Starting mission"
	# Reset mission set to first (0) waypoint
	vehicle.commands.next=0
	time.sleep(1)
	
	# Set mode to AUTO to start mission
	vehicle.mode = VehicleMode("AUTO")
	
	while rospy.get_param('simulation_running') is True:
		nextwaypoint=vehicle.commands.next
		if nextwaypoint==6:
			vehicle.mode = VehicleMode("RTL")
			break
		else:
			print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint(vehicle))
		time.sleep(1)
	
	
	if rospy.get_param('collison_detected') is True:
		print('Setting vehicle mode to HOLD')
		vehicle.mode = VehicleMode("HOLD")
	else:
		while vehicle.mode != VehicleMode("HOLD"):
			print 'Returning to launch'
			time.sleep(1)
	
	print('mission complete!')
	rospy.set_param('mission_success', True)
	print('This sim is done!')
	scan_sub.unregister()
	

### Set up ROS subscribers and publishers ###
rospy.init_node('rover_dronekit_controller',anonymous=False)
eval_start_sub = rospy.Subscriber('evaluation_start', std_msgs.msg.Empty, eval_start_callback)
obstacle_avoidance_cmds_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

rospy.spin()
