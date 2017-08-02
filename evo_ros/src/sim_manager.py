#!/usr/bin/env python
#
# Sim Manager
#
# Checks ending conditions of the simulation, gathers results, and 
#	publishes them on the sim_result topic
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
import sys

import std_msgs.msg

from argparse import RawTextHelpFormatter
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetWorldProperties
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState

max_sim_time = 240

# Percent of the maze that is complete
percent_complete = 0

# Time required to complete the maze
evaluation_result = ''

# The last region that the vehicle was in
last_known_region = 0

# Distance of scan at which a collision is determined
collision_distance = 0.35

# Boolean representing if an ending condition has been triggered or not
simulation_end = False


def software_ready_callback(data):
	print("Software is ready! Starting sim....")
	
	global simulation_end
	simulation_end = False
	time_fitness = 0
	
	# Get the beginning time for the simulation
	# Error handling for if a required process crashes
	#	Mainly for Gazebo, which has a tendency to crash during
	#	long runs
	try:
		begin_time = getWorldProp().sim_time 
	except:
		print('Required processes has failed. Sending reset message to software manager')
		sim_result_pub.publish(-2)
	
	# Send a ready message on the evaluation start topic to let controller
	#	know that the simulation enviroment is ready
	rospy.set_param('simulation_running', True)
	eval_start_pub.publish(std_msgs.msg.Empty())


	# Wait until a simulation end event is triggered
	#	or for the max simulation time to be reached
	sim_timeout = False
	while simulation_end is False:
		# Error handling for if a required process crashes
		#	Mainly for Gazebo, which has a tendency to crash during
		#	long runs
		try:
			current_time = getWorldProp().sim_time
		except:
			print('Required processes has failed. Sending reset message to software manager')
			sim_result_pub.publish(-2)
		if (current_time - begin_time) > max_sim_time:
			print("Simulation timed out.")
			simulation_end = True
			sim_timeout = True
		time.sleep(0.1)
	
	rospy.set_param('simulation_running', False)
	
	# If the vehicle was able to finish successfully, give it a time bonus
	#	Else send back a result of -1 indicating a collision
	#	Or -2 indicating that the simulation took too long to finish sucessfully
	global percent_complete
	if percent_complete == 100:
		current_time = getWorldProp().sim_time 
		total_sim_time = current_time - begin_time
		time_fitness = (max_sim_time - total_sim_time) / max_sim_time
	else:
		if sim_timeout == True:
			time_fitness = -2
		else:
			time_fitness = -1
		

	
	# Reset simulation
	print("Attempting to reset...")
	resetWorld()
	time.sleep(3)
	print("Reset!")
	
	# Send simulation results
	rospy.set_param('percent_complete', percent_complete)
	sim_result_pub.publish(time_fitness)
	

# Old lidar based method of detecting a collision
def scan_callback(data):
	global simulation_end
	
	if rospy.get_param('simulation_running') is True:
		for current_range in data.ranges[len(data.ranges)*2/5:len(data.ranges)*3/5]:
			if current_range < collision_distance:
				print('Collision detected!')
				simulation_end = True

				
def contact_test(contacts):
	global simulation_end
	
	if rospy.get_param('simulation_running') is True:
		# Check for contact states
		#	If none, pass
		#	If any contact, mark a collision
		if not contacts.states:
			pass
		else:
			print('Collision detected!')
			simulation_end = True


def regions_callback(msg):
	global last_known_region
	global percent_complete
	global simulation_end
	
	if rospy.get_param('simulation_running') is True:
		if msg.data == last_known_region:
			#print('pass')
			pass
		else:
			last_known_region = msg.data
			percent_complete = float(msg.data)
			print('Percent complete: {}'.format(percent_complete))
			if percent_complete == 100:
				print('Vehicle has finished simulation successfully!')
				simulation_end = True
	

### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""
Checks ending conditions of the simulation, gathers results, and publishes them on the sim_result topic
	""", formatter_class=RawTextHelpFormatter)

args= parser.parse_args()


### Wait for Gazebo Services ###
print('Waiting for gazebo services')
rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/reset_world')
rospy.wait_for_service('/gazebo/reset_simulation')
rospy.wait_for_service('/gazebo/pause_physics')
rospy.wait_for_service('/gazebo/unpause_physics')


### Set up ROS service Proxies ###
getWorldProp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)


### Set up ROS subscribers and publishers ###
rospy.init_node('sim_manager',anonymous=False)
rospy.set_param('simulation_running', False)
rospy.set_param('percent_complete', 0)
eval_start_pub = rospy.Publisher('evaluation_start', std_msgs.msg.Empty, queue_size=1)
sim_result_pub = rospy.Publisher('evaluation_result', std_msgs.msg.Float64, queue_size=1)
sim_start_sub = rospy.Subscriber('software_ready', std_msgs.msg.Empty, software_ready_callback)
#scan_sub = rospy.Subscriber("/scan", LaserScan,scan_callback)
contact_sensor_sub = rospy.Subscriber("/chassis_contact_sensor_state", ContactsState,contact_test)
region_sub = rospy.Subscriber('ros_regions', std_msgs.msg.String, regions_callback, queue_size=1)
print("Done!")
rospy.spin()


