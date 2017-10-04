#!/usr/bin/env python
#
#
#from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import sys
import rospy
from pymavlink import mavutil
from dronekit_functions import get_location_metres, get_distance_metres, distance_to_current_waypoint, download_mission, adds_square_mission, is_vehicle_home
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from mavros_msgs.msg import OverrideRCIn
import message_filters
import std_msgs.msg
import numpy as np
from obstacle_avoidance_functions import partition_vision, check_vision, parse_genome, findMiddle, ErleRover_Obstacle_Avoidance, sonar_avoidance

import mav_msgs.msg as mav_msgs
import collections


history_queue = collections.deque(maxlen=200) 



# Temp function
#   Remove when integrated into evo-ros
def load_genome():
	ind = {'id':0,
		'genome':{
			'physical':[
				{'sensor':'sonar1', 'pos':[0.25, -0.1, 0.17], 'orient':[0, 0, -20]},
				{'sensor':'sonar2', 'pos':[0.25, 0.1, 0.17], 'orient':[0, 0, 20]}
			],
			'behavioral':[
			]
			},
		'fitness':-1.0,
		'generation':0
		}
	
	rospy.set_param('vehicle_genome', ind)


# Get Sonar Angles
#   Pulls the vehicle genome from the ros param and stores all of the sonar angles into a dict
def get_sonar_angles():
	sonar_angles_dict = {}
	
	### Get vehicle genome ###
	genome = rospy.get_param('vehicle_genome')

	# Search for sonars in the physical genome traits and add any sonar angles
	for genome_trait in genome['genome']['physical']:
		# check for sonar
		if 'sonar' in genome_trait['sensor']:
			# Add angle
			sonar_angles_dict[genome_trait['sensor']] = genome_trait['orient'][2]

	return sonar_angles_dict





# Sonar Callback
#   Collect sonar range data that is present into dict
#   and pass to the sonar avoidance function to get current nav_cmds
#   Send nav_cmds to MAVROS
def sonar_callback(sonar1 = '', sonar2 = '', sonar3 = ''):
	sonar_ranges = {}
	range_max = 5
	if sonar1 is not '':
		#print('Sonar1: {}'.format(sonar1.range))
		range_max = sonar1.max_range
		sonar_ranges['sonar1'] = sonar1.range
	if sonar2 is not '':
		#print('Sonar2: {}'.format(sonar2.range))
		sonar_ranges['sonar2'] = sonar2.range
	if sonar3 is not '':
		#print('Sonar3: {}'.format(sonar3.range))
		sonar_ranges['sonar3'] = sonar3.range

	# Use obstacle avoidance algorithm
	nav_cmds = sonar_avoidance(sonar_ranges, sonar_angles, range_max)
	#nav_cmds = test_avoidance(sonar_ranges, range_max)
	
	
	msg = OverrideRCIn()
	msg.channels[0] = nav_cmds['yaw']
	msg.channels[1] = 0
	msg.channels[2] = nav_cmds['throttle']
	msg.channels[3] = 0
	msg.channels[4] = 0
	msg.channels[5] = 0
	msg.channels[6] = 0
	msg.channels[7] = 0

	obstacle_avoidance_cmds_pub.publish(msg)
	print(nav_cmds)




### Set up ROS subscribers and publishers ###
rospy.init_node('sonar_obstacle_avoidance',anonymous=False)
obstacle_avoidance_cmds_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

### Load test genome into ros param ###
load_genome()

### Detect which sonars are on the rover ### 
sonar_sub_list = []
sonar1_sub = ''
sonar2_sub = ''
sonar3_sub = ''
topics_list = rospy.get_published_topics()


if ['/sonar1', 'sensor_msgs/Range'] in topics_list:
	print('Adding sonar 1')
	sonar1_sub = message_filters.Subscriber('/sonar1', Range)
	sonar_sub_list.append(sonar1_sub)
if ['/sonar2', 'sensor_msgs/Range'] in topics_list:
	print('Adding sonar 2')
	sonar2_sub = message_filters.Subscriber('/sonar2', Range)
	sonar_sub_list.append(sonar2_sub)
if ['/sonar3', 'sensor_msgs/Range'] in topics_list:
	print('Adding sonar 3')
	sonar3_sub = message_filters.Subscriber('/sonar3', Range)
	sonar_sub_list.append(sonar3_sub)

print('Detected sonars: {}'.format(sonar_sub_list))

### Get angles of sonars on the vehicle ###
sonar_angles = get_sonar_angles()
print('Sonar angles: {}'.format(sonar_angles))


### Set up a single callback function for all sonars ###
#ts = message_filters.ApproximateTimeSynchronizer(sonar_sub_list, 10, 0.15)
ts = message_filters.TimeSynchronizer(sonar_sub_list, 10)
ts.registerCallback(sonar_callback)

rospy.spin()
