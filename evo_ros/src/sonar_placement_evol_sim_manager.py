#!/usr/bin/env python
#
# Sonar Placement without failure Sim Manager
#
# Checks ending conditions of the simulation, gathers results, and 
#	publishes them on the sim_result topic
#
# GAS 2017-10-06

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
import yaml

import std_msgs.msg

from argparse import RawTextHelpFormatter
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetWorldProperties
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState


# Percent of the maze that is complete
percent_complete = 0

# Time required to complete the maze
evaluation_result = ''

# Waypoints that the rover has made it to
waypoint_history = []

# Distance of scan at which a collision is determined
collision_distance = 0.35

# Boolean representing if an ending condition has been triggered or not
simulation_end = False


def software_ready_callback(data):
	print("Software is ready! Starting sim....")
	
	global simulation_end
	global percent_complete
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
	rospy.set_param('collison_detected', False)
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
		if (current_time - begin_time) > MAX_SIM_TIME:
			print("Simulation timed out.")
			simulation_end = True
			sim_timeout = True
			
		if rospy.get_param('mission_success') is True:
			percent_complete = 100
			simulation_end = True
			
		time.sleep(0.1)
	
	rospy.set_param('simulation_running', False)
	
	# If the vehicle was able to finish successfully, give it a time bonus
	#	Else send back a result of -1 indicating a collision
	#	Or -2 indicating that the simulation took too long to finish sucessfully
	if percent_complete == 100:
		current_time = getWorldProp().sim_time 
		total_sim_time = current_time - begin_time
		time_fitness = (MAX_SIM_TIME - total_sim_time) / MAX_SIM_TIME
	else:
		if sim_timeout == True:
			time_fitness = -2
		else:
			time_fitness = -1
		

	time.sleep(3)
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
				rospy.set_param('collison_detected', True)

				
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

def waypoint_callback(msg):
	global waypoint_history
	global percent_complete
	
	if msg not in waypoint_history:
		waypoint_history.append(msg)
		print('Made it to waypoint {}'.format(msg))
		percent_complete += 20
		print('Percent complete: {}'.format(percent_complete))
	
	


### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""
Checks ending conditions of the simulation, gathers results, and publishes them on the sim_result topic
	""", formatter_class=RawTextHelpFormatter)
parser.add_argument('-c', '--config', type=str, help='The configuration file that is to be used')
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
args= parser.parse_args()

### Start Configuration ###

# Use default config file unless one is provided at command line
config_file_name = 'default_config.yml'
if args.config is not None:
	config_file_name = args.config

if args.debug:
	print('Configuration file being used: \n\t {}'.format(os.path.dirname(os.path.abspath(__file__)) + '/../config/{}'.format(config_file_name)))

# Open Config File
with open(os.path.dirname(os.path.abspath(__file__)) + '/../config/{}'.format(config_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

MAX_SIM_TIME = cfg['rover_sim_manager']['MAX_SIM_TIME']

# If debugging print configuration settings to screen
if args.debug:
	print("""\n\tConfiguration Settings...
		MAX_SIM_TIME: {}
		""".format(MAX_SIM_TIME))

### End Configuration ###


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
rospy.set_param('collison_detected', False)
rospy.set_param('mission_success', False)
rospy.set_param('percent_complete', 0)
eval_start_pub = rospy.Publisher('evaluation_start', std_msgs.msg.Empty, queue_size=1)
sim_result_pub = rospy.Publisher('evaluation_result', std_msgs.msg.Float64, queue_size=1)
sim_start_sub = rospy.Subscriber('software_ready', std_msgs.msg.Empty, software_ready_callback)
contact_sensor_sub = rospy.Subscriber("/chassis_contact_sensor_state", ContactsState,contact_test)
waypoint_sub = rospy.Subscriber('rover/waypoints', std_msgs.msg.Float64, waypoint_callback)
print("Done!")
rospy.spin()


