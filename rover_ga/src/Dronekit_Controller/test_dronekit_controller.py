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
from dronekit_functions import get_location_metres, get_distance_metres, distance_to_current_waypoint, download_mission, adds_square_mission, is_vehicle_home, angle_to_current_waypoint
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from mavros_msgs.msg import OverrideRCIn
import std_msgs.msg
import numpy as np
import message_filters
from obstacle_avoidance_functions import partition_vision, check_vision, parse_genome, findMiddle, ErleRover_Obstacle_Avoidance, sonar_avoidance

import mav_msgs.msg as mav_msgs
import collections
history_queue = collections.deque(maxlen=200)
history_queue.append(1500)

last_vehicle_mode = VehicleMode("AUTO")

def rover_commands_cb(data):
	global history_queue
	global vehicle
	
	print('Angle to WP: {}'.format(angle_to_current_waypoint(vehicle)))
	print('Vehicle heading: {}'.format(vehicle.heading))
	if vehicle.mode == VehicleMode("AUTO"):
		yaw = data.motor_speed[0] + 1000
		history_queue.append(yaw)
		#print(sum(history_queue)/float(len(history_queue)))


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
	
	global vehicle
	global last_vehicle_mode
	global history_queue
	
		
	sonar_ranges = {}
	range_max = 4
	hybrid_zone_cutoff = 2
	if sonar1 is not '':
		#print('Sonar1: {}'.format(sonar1.range))
		range_max = sonar1.max_range
		sonar_ranges['sonar1'] = sonar1.range - 0.5
	if sonar2 is not '':
		#print('Sonar2: {}'.format(sonar2.range))
		sonar_ranges['sonar2'] = sonar2.range
	if sonar3 is not '':
		#print('Sonar3: {}'.format(sonar3.range))
		sonar_ranges['sonar3'] = sonar3.range

	
	# Check to see if an object is in the path of the rover
	if all(i >= range_max for i in sonar_ranges.values()):
		# All is good
		if(vehicle.mode == VehicleMode("MANUAL")):
			#print('Clear of obstacle! Switching to {}'.format(last_vehicle_mode))
			vehicle.mode = last_vehicle_mode
	else:
		if(vehicle.mode != VehicleMode("MANUAL")):
			#print('Object detected! Obstacle avoidance engaged!')
			last_vehicle_mode = vehicle.mode
		vehicle.mode = VehicleMode("MANUAL")
		
		
		# Use obstacle avoidance algorithm
		nav_cmds = sonar_avoidance(sonar_ranges, sonar_angles, range_max)
		
		
		# Use hybrid of O.A. and W.P.F. if obstacles are further out in the detection range
		if min(sonar_ranges.values()) > hybrid_zone_cutoff:
			w_p_f_cmds = sum(history_queue)/len(history_queue)
			
			weight = min(sonar_ranges.values()) / range_max
			
			# W.P and object are on the right
			if (w_p_f_cmds > 1500 and nav_cmds['yaw'] < 1500):
				nav_cmds['yaw'] = (400 * weight) + 1500
			
			# W.P. and object are on the left
			if (w_p_f_cmds < 1500 and nav_cmds['yaw'] > 1500):
				nav_cmds['yaw'] = 1500 - (400 * weight)
				
				
			
			weight = min(sonar_ranges.values()) / range_max
			
			#nav_cmds['yaw'] = weight * w_p_f_cmds + (1 - weight) * nav_cmds['yaw']
			
			
			#print('Use hybrid!')
			#print('Yaw: {}'.format(nav_cmds['yaw']))			
		
		
					
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
		#print(nav_cmds)



connection_string = '127.0.0.1:14551'


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

#Subscribe to vehicle commands
rover_commands_sub = rospy.Subscriber('/rover/command/motor_speed',  mav_msgs.CommandMotorSpeed, rover_commands_cb)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=0
time.sleep(1)

### Set up a single callback function for all sonars ###
#ts = message_filters.ApproximateTimeSynchronizer(sonar_sub_list, 10, 0.15)
ts = message_filters.TimeSynchronizer(sonar_sub_list, 10)
ts.registerCallback(sonar_callback)

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

while True:
	nextwaypoint=vehicle.commands.next
	if nextwaypoint==6:
		vehicle.mode = VehicleMode("RTL")
		break
	else:
		print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint(vehicle))
	time.sleep(1)

time.sleep(1)

while vehicle.mode != VehicleMode("HOLD"):
	print 'Returning to launch'
	time.sleep(1)


print('mission complete!')
rospy.spin()
