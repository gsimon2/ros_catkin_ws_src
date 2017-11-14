#!/usr/bin/env python
#
# Sonar Filter
#	Sonar topics are passed through this node so that their signals can be modified before they reach the conroller
#
# 10-31-17

import os
import rospy
import sys
import argparse
import random
import datetime
import yaml

from sensor_msgs.msg import Range
import message_filters

SENSOR_KNOCKOUT = False
KNOCKOUT_TYPE = 'complete'


KNOCKOUT_POINT = 20
KNOCKOUT_INTERVAL = 15
SENSOR_STATUS = 'ON'
KNOCKOUT_SENSOR = ''
GENERATION = -1
KNOCKOUT_TIME = 0


# Percent complete knockout
#	Starting at KNOCKOUT_POINT one sensor fails and then will toggle between working and failing states every KNOCKOUT_INTERVAL
def percent_complete_knockout(sonar1 = '', sonar2 = '', sonar3 = '', sonar4 = '', sonar5 = '', sonar6 = '', sonar7 = '', sonar8 = '', sonar9 = '', sonar10 = ''):
	global KNOCKOUT_POINT
	global SENSOR_STATUS
	
	percent_complete = rospy.get_param('percent_complete') 
	for j in range (1,11):
		current_sonar = 'sonar' + str(j)
		current_sonar_pub = current_sonar + '_pub'
		
		if eval(current_sonar) is not '':
			
			# Update the knock out status if the vehicle is past the knockout point
			if percent_complete >= KNOCKOUT_POINT:
				
				# Toggle whether this sensor is working or not
				if SENSOR_STATUS == 'ON':
					SENSOR_STATUS = 'OFF'
					
					if args.debug:
						print('Turning off: {}'.format(KNOCKOUT_SENSOR))
						
				else:
					SENSOR_STATUS = 'ON'
					
					if args.debug:
						print('Turning on: {}'.format(KNOCKOUT_SENSOR))
				
				# Update the next time that this sensor state will be toggled
				KNOCKOUT_POINT += KNOCKOUT_INTERVAL
				
				if args.debug:
						print('Next state change: {}'.format(KNOCKOUT_POINT))
				
			# If the sensor is knocked out, clear the range data before publishing
			if current_sonar == KNOCKOUT_SENSOR:
				if SENSOR_STATUS == 'OFF':
					eval(current_sonar).range = float(-1)
					
			eval(current_sonar_pub).publish(eval(current_sonar))


# Single complete knockout
#	Starting at a simulated time  a sensor will be knocked out and remain off for the rest of the run
def single_complete_knockout(sonar1 = '', sonar2 = '', sonar3 = '', sonar4 = '', sonar5 = '', sonar6 = '', sonar7 = '', sonar8 = '', sonar9 = '', sonar10 = ''):
	global SENSOR_STATUS
	running_time = rospy.get_param('running_time') 
	for j in range (1,11):
		current_sonar = 'sonar' + str(j)
		current_sonar_pub = current_sonar + '_pub'
		
		if eval(current_sonar) is not '':
			
			# Update the knock out status if the vehicle is past the knockout point
			if running_time >= KNOCKOUT_TIME:
				# Toggle whether this sensor is working or not
				if SENSOR_STATUS == 'ON':
					SENSOR_STATUS = 'OFF'
					
					if args.debug:
						print('Turning off: {}'.format(KNOCKOUT_SENSOR))
				
			# If the sensor is knocked out, clear the range data before publishing
			if current_sonar == KNOCKOUT_SENSOR:
				if SENSOR_STATUS == 'OFF':
					eval(current_sonar).range = float(-1)
					
			eval(current_sonar_pub).publish(eval(current_sonar))		

# Transient Failure Knockout
#	At a simulated time a snesr will be knocked out and will then toggle between working and not
def transient_knockout(sonar1 = '', sonar2 = '', sonar3 = '', sonar4 = '', sonar5 = '', sonar6 = '', sonar7 = '', sonar8 = '', sonar9 = '', sonar10 = ''):
	global KNOCKOUT_TIME
	global SENSOR_STATUS
	
	running_time = rospy.get_param('running_time') 
	for j in range (1,11):
		current_sonar = 'sonar' + str(j)
		current_sonar_pub = current_sonar + '_pub'
		
		if eval(current_sonar) is not '':
			
			# Update the knock out status if the vehicle is past the knockout point
			if running_time >= KNOCKOUT_TIME:
				
				# Toggle whether this sensor is working or not
				if SENSOR_STATUS == 'ON':
					SENSOR_STATUS = 'OFF'
					
					if args.debug:
						print('Turning off: {}'.format(KNOCKOUT_SENSOR))
						
				else:
					SENSOR_STATUS = 'ON'
					
					if args.debug:
						print('Turning on: {}'.format(KNOCKOUT_SENSOR))
				
				# Update the next time that this sensor state will be toggled
				KNOCKOUT_TIME += KNOCKOUT_INTERVAL
				
				if args.debug:
						print('Next state change: {}'.format(KNOCKOUT_TIME))
				
			# If the sensor is knocked out, clear the range data before publishing
			if current_sonar == KNOCKOUT_SENSOR:
				if SENSOR_STATUS == 'OFF':
					eval(current_sonar).range = float(-1)
					
			eval(current_sonar_pub).publish(eval(current_sonar))

### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""Bypass for sonar topics so noise can be added or sonar signals can be cutout during the run""")
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal')
parser.add_argument('-k','--knockout',action='store_true', help='Knocks out a sensor partial through a simulation')
parser.add_argument('-c', '--config', type=str, help='The configuration file that is to be used')
args= parser.parse_args()


# Use default config file unless one is provided at command line
config_file_name = 'default_config.yml'
if args.config is not None:
	config_file_name = args.config

if args.debug:
	print('Configuration file being used: \n\t {}'.format(os.path.dirname(os.path.abspath(__file__)) + '/../config/{}'.format(config_file_name)))

# Open Config File
with open(os.path.dirname(os.path.abspath(__file__)) + '/../config/{}'.format(config_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)


# Sensor knockout
#	If sensors will fail during the run
if args.knockout:
	SENSOR_KNOCKOUT = True
else:
	SENSOR_KNOCKOUT = cfg['sonar_filter']['SENSOR_KNOCKOUT']	
	
# Type of failure
KNOCKOUT_TYPE = cfg['sonar_filter']['KNOCKOUT_TYPE']	


### Set up ROS subscribers and publishers ###
rospy.init_node('sonar_filter',anonymous=False)
rospy.set_param('percent_complete', 0)
rospy.set_param('running_time', 0)

sonar1_pub = rospy.Publisher('/sonar_filtered1', Range, queue_size=10)
sonar2_pub = rospy.Publisher('/sonar_filtered2', Range, queue_size=10)
sonar3_pub = rospy.Publisher('/sonar_filtered3', Range, queue_size=10)
sonar4_pub = rospy.Publisher('/sonar_filtered4', Range, queue_size=10)
sonar5_pub = rospy.Publisher('/sonar_filtered5', Range, queue_size=10)
sonar6_pub = rospy.Publisher('/sonar_filtered6', Range, queue_size=10)
sonar7_pub = rospy.Publisher('/sonar_filtered7', Range, queue_size=10)
sonar8_pub = rospy.Publisher('/sonar_filtered8', Range, queue_size=10)
sonar9_pub = rospy.Publisher('/sonar_filtered9', Range, queue_size=10)
sonar10_pub = rospy.Publisher('/sonar_filtered10', Range, queue_size=10)


### Detect which sonars are on the rover ### 
sonar_sub_list = []

topics_list = rospy.get_published_topics()

for j in range(1,11):
	sonar_topic = '/sonar' + str(j)
	current_sonar_pub = 'sonar' + str(j) + '_pub'
	if [sonar_topic, 'sensor_msgs/Range'] in topics_list:
		sonar_sub_list.append(message_filters.Subscriber(sonar_topic, Range))
	else:
		# Only publish filtered topics for sonars that are present
		eval(current_sonar_pub).unregister()

### Set up a single callback function for all sonars ###
ts = message_filters.TimeSynchronizer(sonar_sub_list, 10)

if KNOCKOUT_TYPE == 'complete':
	ts.registerCallback(single_complete_knockout)
elif KNOCKOUT_TYPE == 'transient':
	ts.registerCallback(transient_knockout)

### If Knockout, Determine which sensor we are knocking out ###
if SENSOR_KNOCKOUT:
	GENERATION = rospy.get_param('generation')
	#today = datetime.date.today().day # Was thinking about seeding with generation * the day number
	random.seed(GENERATION)
	KNOCKOUT_TIME = rospy.get_param('running_time') + random.randrange(40,80,1)
	number_of_sensors = rospy.get_param('vehicle_genome')['genome']['num_of_sensors']
	knockout_number = (GENERATION % number_of_sensors) + 1
	KNOCKOUT_SENSOR = 'sonar' + str(knockout_number)
	
	if args.debug:
		print('Gen: {} \t Num sensors: {} \t knockout num: {}'.format(GENERATION, number_of_sensors, knockout_number))
		print('knockout time: {} '.format(KNOCKOUT_TIME))
		print('Type of failure: {}'.format(KNOCKOUT_TYPE))
	
rospy.spin()
