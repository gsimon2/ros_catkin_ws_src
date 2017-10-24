#!/usr/bin/env python

### Code comes from:
# 	http://python.dronekit.io/examples/mission_basic.html
###
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import sys
import rospy
import tf
import argparse
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

from gazebo_msgs.msg import LinkStates

from rover_ga.msg import waypoint

last_vehicle_mode = VehicleMode("AUTO")
last_heading = 0



# Link States Callbakc
#	Get the orientation of the chassis of the rover from Gazebo Linked States topic and calculates its heading 
def link_states_callback(data):
	global last_heading
	# Get the index of the base_link
	# data.name contains all link names found in the URDF (or SDF) file
	
	base_link_index = data.name.index('rover::chassis')
	base_link_position = data.pose[base_link_index].position
	base_link_orientation = data.pose[base_link_index].orientation
	
	# Convert the quaternion data to roll, pitch, yaw
	quat = (
		base_link_orientation.x,
		base_link_orientation.y,
		base_link_orientation.z,
		base_link_orientation.w)
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
	
	# Convert heading to same notation that bearing is in
	#	True east = 0 degrees, then count up clock wise so South = 90, West = 180, North = 270
	if math.degrees(yaw) < 0:
		last_heading = math.degrees(yaw) * -1
	else:
		last_heading = 360 - math.degrees(yaw)
	#print 'heading           :' + str(math.degrees(yaw))


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
def sonar_callback(sonar1 = '', sonar2 = '', sonar3 = '', sonar4 = '', sonar5 = '', sonar6 = '', sonar7 = '', sonar8 = '', sonar9 = '', sonar10 = ''):
	
	global vehicle
	global last_vehicle_mode
	global history_queue
	global last_heading
	
	
	ang = angle_to_current_waypoint(vehicle)
	#print('Bearing: {} \t Heading: {}'.format(ang, last_heading))
		
	sonar_ranges = {}
	range_max = 2.5
	hybrid_zone_cutoff = 2
	
	# Read any sonar data available and put it into a single dict called sonar_ranges
	#	Also have a modifier for range_max - This is in case the max sensing capabilities are changed in the URDF file
	#	It is -0.5 because at the default angle that sonars are passed on the rover, they catch the ground at the very end
	#	of the sensing range. We want to ignore the ground readings while still picking up small objects
	for j in range (1,11):
		current_sonar = 'sonar' + str(j)
		if eval(current_sonar) is not '':
			range_max = eval(current_sonar).max_range  - 0.5
			sonar_ranges[current_sonar] = eval(current_sonar).range
			
			
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
rospy.Subscriber('gazebo/link_states', LinkStates, link_states_callback)
waypoint_pub = rospy.Publisher('/rover/waypoints', waypoint, queue_size=10)


### Detect which sonars are on the rover ### 
sonar_sub_list = []

time.sleep(3)

topics_list = rospy.get_published_topics()

for j in range(1,11):
	sonar_topic = '/sonar' + str(j)
	#print('Sonar topic {}'.format(sonar_topic))
	if [sonar_topic, 'sensor_msgs/Range'] in topics_list:
		#print('Adding sonar {}'.format(j))
		sonar_sub_list.append(message_filters.Subscriber(sonar_topic, Range))

#print('Detected sonars: {}'.format(sonar_sub_list))

### Get angles of sonars on the vehicle ###
sonar_angles = get_sonar_angles()
#print('Sonar angles: {}'.format(sonar_angles))

time.sleep(2)

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout = 300)

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

### Set up a single callback function for all sonars ###
#ts = message_filters.ApproximateTimeSynchronizer(sonar_sub_list, 10, 0.15)
ts = message_filters.TimeSynchronizer(sonar_sub_list, 10)
ts.registerCallback(sonar_callback)

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# carry out the mission and publish updates about what the last waypoint we visited was and how far we have until the next one
while vehicle.mode != VehicleMode("HOLD"):
	msg = waypoint()
	nextwaypoint=vehicle.commands.next
	msg.last_visited_waypoint = nextwaypoint - 1
	dist_to_current_waypoint = distance_to_current_waypoint(vehicle)
	msg.distance_to_next_waypoint = dist_to_current_waypoint
	waypoint_pub.publish(msg)
	#print 'Distance to waypoint (%s): %s' % (nextwaypoint, dist_to_current_waypoint)
	time.sleep(0.25)

print 'Returning to launch'
vehicle.mode = VehicleMode("RTL")
time.sleep(1)

# Return to launch location and publish update messages about how far away we are
while vehicle.mode != VehicleMode("HOLD"):
	msg = waypoint()
	msg.last_visited_waypoint = 4
	dist_to_home = get_distance_metres(vehicle.location.global_frame, vehicle.home_location)
	msg.distance_to_next_waypoint = dist_to_home
	waypoint_pub.publish(msg)
	#print 'Returning to launch'
	time.sleep(0.25)

# Made it to launch (home) location. Publish a message saying so
msg = waypoint()
msg.last_visited_waypoint = 5
msg.distance_to_next_waypoint = 0
waypoint_pub.publish(msg)
print('mission complete!')
rospy.spin()
