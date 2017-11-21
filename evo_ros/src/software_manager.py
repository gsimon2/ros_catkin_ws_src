#!/usr/bin/env python
#
# Software Manager
#
# Responsible for two main functions:
#	-Spawning all required software for evaluating the genome
#		(Gazebo, other ROS nodes, and Ardupilot if needed)
#	-Reading the physical (morphology) aspect of the genome and modifying
#		the vehicle .urdf file as needed
#
# GAS 2017-07-19

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

from add_sensor_functions import add_lidar_rover
from add_sensor_functions import add_sonar_rover
from add_sensor_functions import copy_base_rover_file
from add_sensor_functions import add_lidar_copter
from add_sensor_functions import copy_base_copter_file
from argparse import RawTextHelpFormatter


last_physical_genome = []

evaluation_result = ''

random.seed(datetime.datetime.now())

# The current generation of genomes that is being evaluated
GENERATION = -1


	
### Simulation Result Callback ###
### 	Collect received sim result data and store it in global var
def sim_result_callback(data):
	global last_physical_genome
	global evaluation_result
	print('Evaluation instance completed with result: {}'.format(data.data))
	# Check when the simulation takes too long to complete and reset the software
	#	For the next received genome
	if data.data == -2:
		print('Reseting stored genome due to software crash')
		last_physical_genome = []
	if data.data == -3:
		print('Reseting stored genome due to sim timeout')
		last_physical_genome = []
	evaluation_result = data.data

### Pick New Launch ###
###		Uses random number to select which launch file is used next 
###		This allows for different worlds to be used during GA runs 
def pick_new_launch():
	world_selection = random.randrange(1,4,1)
	if world_selection == 1:
		new_launch = 'maze_1.launch'
	if world_selection == 2:
		new_launch = 'maze_2.launch'
	if world_selection == 3:
		new_launch = 'maze_3.launch'
	return new_launch
	
### Shutdown Hook ###
### 	This is the last function executed before the script exits 
###		Cleans up all processes spawned during evaluation 
def shutdown_hook():
	cmd_str = "killall -9 gzserver gzclient roscore rosmaster rosout mavproxy.py xterm"
	os.system(cmd_str)
	cmd_str = "pkill -1 -f {}".format(CONTROLLER_SCRIPT)
	os.system(cmd_str)
	cmd_str = "pkill -1 -f region_events_node"
	os.system(cmd_str)
	cmd_str = "pkill -1 -f transporter.py"
	os.system(cmd_str)
	cmd_str = "pkill -1 -f {}".format(SIM_MANAGER_SCRIPT)
	os.system(cmd_str)
	cmd_str = "pkill -9 -f mavros_node"
	os.system(cmd_str)
	time.sleep(2)
	print('Tear down complete. Exiting...')

### Software Setup ###
### 	This sets up all of the software that is need to run an evaluation,
###		but must be torn down on a morphological change to the robot.
###		Includes:
###			-Mavproxy (Spawns ardupilot and MAVROS)
###			-The ROS vehicle controller node
###			-The ROS launch file
###		Also rewrites the URDF file for the robot on morphological change.
def software_setup(data):
	
	# Tear down this simulation instance
	cmd_str = "pkill -1 -f {}".format(SIM_MANAGER_SCRIPT)
	os.system(cmd_str)
	time.sleep(3)
	cmd_str = "killall -9 gzserver gzclient mavproxy.py xterm"
	os.system(cmd_str)
	cmd_str = "pkill -9 -f mavros_node"
	os.system(cmd_str)
	if args.debug is False:
		try:
			mavproxy.kill()
			launch_file.kill()
			controller_script.kill()
			mavproxy.wait()
			launch_file.wait()
			controller_script.wait()
		except Exception:
			pass
	time.sleep(3)


	# Clear the Gazebo logs 
	cmd_str =  "rm -rf ~/.gazebo/log/*"
	os.system(cmd_str)
	
	# Get host name of machine #
	str_host_name = socket.gethostname()

	### Set up vehicle URDF files ###
	###		First make a copy of the base_vehicle URDF file with the name
	###			of the local machine appending to it
	###		Second open the URDF file and add any sensors that are defined
	###			in the recv'd vehicle genome
	if 'rover' in VEHICLE:
		#Create a copy of the base rover file for this instance
		str_vehicle_file = copy_base_rover_file(str_host_name)
		
		#Build rover sensors based off recveived genome
		for genome_trait in data['genome']['physical']:
			print('{}\n'.format(genome_trait))
			if 'lidar' in genome_trait['sensor']:
				print("Adding a lidar sensor to the rover")
				#Add sensors based off genome
				add_lidar_rover(str_vehicle_file, genome_trait['pos'], genome_trait['orient'])
				
			elif 'sonar' in genome_trait['sensor']:
				print("Adding a sonar sensor to the rover")
				#Add sensors based off genome
				add_sonar_rover(str_vehicle_file, genome_trait['pos'], genome_trait['orient'], genome_trait['sensor'])
	elif 'copter' in VEHICLE:
		#Create a copy of the base rover file for this instance
		str_vehicle_file = copy_base_copter_file(str_host_name)
		
		#Build copter sensors based off recveived genome
		for genome_trait in data['genome']['physical']:
			print('{}\n'.format(genome_trait))
			if 'lidar' in genome_trait['sensor']:
				print("Adding a lidar sensor to the copter")
				#Add sensors based off genome
				add_lidar_copter(str_vehicle_file, genome_trait['pos'], genome_trait['orient'])
		
		# Pass a null value for the model to the launch file which causes the default specifed to be used
		#	For the copter this is a model with a Gazebo wrapper which will open the file we modified in the
		# 	above functions
		str_vehicle_file = ""
	else:
		print('Invalid vehicle selection during the set up vehicle URDF file section!')
		sys.exit()

	time.sleep(1)
	
	### Start all needed processes ###
	# Start MAVProxy
	if args.debug:
		os.system("xterm -title 'MAVProxy' -hold  -e '{}'&".format(MAVPROXY_CMD_STR))
	else:
		mavproxy = subprocess.Popen(MAVPROXY_CMD_STR, stdout=subprocess.PIPE, shell=True)
	
	#Give time to start up Mavproxy and Ardupilot (takes a while since Ardupilots sim_vehicle script calls xterm to start the ardupilot scripts)
	str_PID = ''
	while(str_PID == ''):
		try:
			str_PID = subprocess.check_output('pidof {}'.format(ARDUPILOT_EXE),stderr=subprocess.STDOUT,shell=True)
		except Exception:
			pass
		time.sleep(0.5)
	print('Started MAVProxy!')
	time.sleep(4)
	
	# Run launch file
	launch_file_cmd_str = 'roslaunch {} {} model:={} gui:={} headless:={}'.format(LAUNCH_FILE_PACKAGE, LAUNCH_FILE, str_vehicle_file, GUI, HEADLESS)
	if args.debug:
		os.system("xterm -hold -e '{}'&".format(launch_file_cmd_str))
	else:
		launch_file = subprocess.Popen(launch_file_cmd_str, stdout=subprocess.PIPE, shell=True)
	print('Started launch file!')
	time.sleep(5)
			
	# Start filter node
	filter_cmd_str = 'rosrun evo_ros sonar_filter.py'
	if args.debug:
		filter_cmd_str += ' -d'
		os.system("xterm -hold -e '{}'&".format(filter_cmd_str))
	else:
		subprocess.Popen(filter_cmd_str, stdout=subprocess.PIPE, shell=True)
	time.sleep(2)
	
	# Start Sim manager node
	if SIM_MANAGER_SCRIPT is not '':
		sim_manager_cmd_str = "rosrun {} {}".format(SIM_MANAGER_PACKAGE, SIM_MANAGER_SCRIPT)
		if args.debug:
			sim_manager_cmd_str = sim_manager_cmd_str + " -d"
			os.system("xterm -hold -e '{}'&".format(sim_manager_cmd_str))
		else:
			sim_manager = subprocess.Popen(sim_manager_cmd_str, stdout=subprocess.PIPE, shell=True)
	
	time.sleep(1)
	#Start controller script
	if CONTROLLER_SCRIPT is not '':
		controller_cmd_str = 'rosrun {} {}'.format(CONTROLLER_SCRIPT_PACKAGE, CONTROLLER_SCRIPT)
		if args.debug:
			os.system("xterm -hold -e '{}'&".format(controller_cmd_str))
		else:
			controller_script = subprocess.Popen(controller_cmd_str, stdout=subprocess.PIPE, shell=True)

	#Give time for everything to start up
	
	if args.less_wait:
		time.sleep(0.5)
	else:
		if args.debug:
			time.sleep(45) #spawning a bunch of xterms for debugging takes longer than subprocesses
		else:
			time.sleep(30)
	

### received_genome_multiple_world_eval_callback ###
### 	Allows for the genome that is received to be evaluated in multiple enviroments
###		A collection of fitnesses is then returned
def received_genome_multiple_world_eval_callback(recv_data):
	global LAUNCH_FILE
	global LAUNCH_FILES_STRING
	global LAUNCH_FILE_PACKAGE
	global NUMBER_OF_WORLDS
	global GENERATION
	global evaluation_result
	global MAX_REAL_TIME
	
	
	#Get genome data
	data = rospy.get_param('vehicle_genome')
	print('Received genome data!')
	
	# Check for ending msg
	if data['id'] == -1:
		print('Received exit message')
		rospy.signal_shutdown('Received exit message from GA')
		return
	
	# Update Generation
	if GENERATION != data['generation']:
		GENERATION = data['generation']
		rospy.set_param('generation', GENERATION)
	
		
	print('Current generation: {}'.format(data['generation']))
	
	# Take the string of launch files and split it into a list
	launch_file_list = LAUNCH_FILES_STRING.strip().split(',')
	
	# Initialize evaluation results list
	eval_results = []
	
	#for i in range(0, NUMBER_OF_WORLDS):
	i = 0
	while i < NUMBER_OF_WORLDS:
		print('Starting Evaluation number: {}'.format(i))
		evaluation_result = ''
		LAUNCH_FILE = launch_file_list[i]
		print('Using launch file: {}'.format(LAUNCH_FILE))
		software_setup(data)
		
		# Notify the sim_manager that all evaluation software is set up
		software_ready_pub.publish(std_msgs.msg.Empty())
		
		print('Done! Entering sleep onto sim evaluation is complete.')
		
		begin_time = datetime.datetime.now()
		
		# Wait for the result for this world (instance is done)
		while evaluation_result == '':
			if ((datetime.datetime.now() - begin_time).total_seconds() > MAX_REAL_TIME):
				evaluation_result = -2
				break
			time.sleep(1)
		print('Out of loop!')
		
		# If gazebo crashes restart without appending the result
		if evaluation_result == -2:
			print('Crashed software. Killing everything and trying again')
			cmd_str = "killall -9 gzserver gzclient mavproxy.py xterm"
			os.system(cmd_str)
			time.sleep(10)
			i -= 1
		else:
			# Push the evalation result and percent complete into the eval_list
			eval_results.append(evaluation_result)
			eval_results.append(rospy.get_param('percent_complete'))
		i+=1

	print('{} of evaluations complete for this genome. \n Total results: {}'.format(NUMBER_OF_WORLDS, eval_results))
	
	
	msg = std_msgs.msg.Float64MultiArray()
	msg.data = eval_results
	sim_result_pub.publish(msg)
	

	

### received_genome Callback ###
###		This is the callback that is used when we are evaluating the received genome in only one enviroment
def received_genome_callback(recv_data):
	global LAUNCH_FILE
	global last_physical_genome
	global GENERATION
	global evaluation_result
	
	# Reset previous eval result
	evaluation_result = ''
	
	#Get genome data
	data = rospy.get_param('vehicle_genome')
	print('Received genome data!')
	
	# Check for ending msg
	if data['id'] == -1:
		print('Received exit message')
		rospy.signal_shutdown('Received exit message from GA')
		return
	
	# Update Generation
	if GENERATION != data['generation']:
		GENERATION = data['generation']
		rospy.set_param('generation', GENERATION)
		
	print('Current generation: {}'.format(data['generation']))
	
	#Check to see if received physical genome is different from last received
	if data['genome']['physical'] != last_physical_genome:
		print("		Received different physical genome!")
		last_physical_genome = data['genome']['physical']
		software_setup(data)
	else:
		print("	 	Same pyhsical genome!")
		software_setup(data)

	# Notify the sim_manager that all evaluation software is set up
	software_ready_pub.publish(std_msgs.msg.Empty())
	
	print('Done! Entering sleep onto sim evaluation is complete.')
	
	
	while evaluation_result == '' or evaluation_result == -2:
		time.sleep(1)
		
		if evaluation_result == -2:
			software_setup(data)
			software_ready_pub.publish(std_msgs.msg.Empty())
	
	print('Simulation complete. Sending result to transporter')
	
	simulation_results = [evaluation_result, rospy.get_param('percent_complete')]
	
	msg = std_msgs.msg.Float64MultiArray()
	msg.data = simulation_results
	sim_result_pub.publish(msg)
	
	
	

### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""
Responsible for two main functions:
	-Spawning all required software for evaluating the genome
		(Gazebo, other ROS nodes, and Ardupilot if needed)
	-Reading the physical (morphology) aspect of the genome and modifying
		the vehicle .urdf file as needed
	""", formatter_class=RawTextHelpFormatter)
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
parser.add_argument('-mw' , '--multiple_worlds', action='store_true', help='Tells software_manager to choose new launch files between generations \n New launch files must be specified in the pick_new_launch function')
parser.add_argument('-gui', '--graphics', action='store_true', help='Start gazebo gui for each simulation')
parser.add_argument('--less_wait',action='store_true',help='Minimize the sleep timers to make running on local machines faster. This option will cause problems when running on remote VMs')
parser.add_argument('-ip' , '--ga_ip_addr', type=str, help='IP address that the GA is running on')
parser.add_argument('-v', '--vehicle', type=str, help='Type of vehicle being used \n\t Accepts: \'rover\' and \'copter\'')
parser.add_argument('-c', '--config', type=str, help='The configuration file that is to be used')
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


# Default GUI state for running Gazebo
#	Note: Headless and GUI should always be opposite of each other
if args.graphics:
	print('Turning on Gazebo GUI')
	HEADLESS = 'false'
	GUI = 'true'
else:
	HEADLESS  = 'true'
	GUI = 'false'


# IP address that external GA is running at
if args.ga_ip_addr is not None:
	GA_IP_ADDR = args.ga_ip_addr
else:
	GA_IP_ADDR = cfg['software_manager']['GA_IP_ADDR']

# The port number that the GA is sending genomes out on
GA_SEND_PORT = cfg['software_manager']['GA_SEND_PORT']

# The port number that the GA is collecting results on
GA_RECV_PORT = cfg['software_manager']['GA_RECV_PORT']

# Max real time seconds that any single evaluation is allowed to run for
MAX_REAL_TIME = cfg['software_manager']['MAX_REAL_TIME']

# The vehicle that is being used. Default is the erle-rover
if args.vehicle is not None:
	VEHICLE = args.vehicle
else:
	VEHICLE = cfg['software_manager']['VEHICLE']

# Mavproxy software
MAVPROXY_CMD_STR = cfg['software_manager']['SCRIPTS'][VEHICLE]['MAVPROXY_CMD_STR']
ARDUPILOT_EXE = cfg['software_manager']['SCRIPTS'][VEHICLE]['ARDUPILOT_EXE']

# Launch file being used
if args.multiple_worlds:
	NUMBER_OF_WORLDS = cfg['software_manager']['MULTIPLE_WORLDS']['NUMBER_OF_WORLDS']
	LAUNCH_FILE_PACKAGE = cfg['software_manager']['MULTIPLE_WORLDS']['LAUNCH_FILE_PACKAGE']
	LAUNCH_FILES_STRING = cfg['software_manager']['MULTIPLE_WORLDS']['LAUNCH_FILES']
	LAUNCH_FILE = LAUNCH_FILES_STRING
else:
	LAUNCH_FILE = cfg['software_manager']['SCRIPTS'][VEHICLE]['LAUNCH_FILE']
	LAUNCH_FILE_PACKAGE = cfg['software_manager']['SCRIPTS'][VEHICLE]['LAUNCH_FILE_PACKAGE']

# Simulaton manager being used
SIM_MANAGER_SCRIPT = cfg['software_manager']['SCRIPTS'][VEHICLE]['SIM_MANAGER_SCRIPT']
SIM_MANAGER_PACKAGE = cfg['software_manager']['SCRIPTS'][VEHICLE]['SIM_MANAGER_PACKAGE']

# Vehicle controller being used
CONTROLLER_SCRIPT = cfg['software_manager']['SCRIPTS'][VEHICLE]['CONTROLLER_SCRIPT']
CONTROLLER_SCRIPT_PACKAGE = cfg['software_manager']['SCRIPTS'][VEHICLE]['CONTROLLER_SCRIPT_PACKAGE']


if args.debug:
	print('Debugging option has been turned on!\n')

# If debugging print configuration settings to screen
if args.debug:
	print("""\n\tConfiguration Settings...
		GA_IP_ADDR: {}
		GA_SEND_PORT: {}
		GA_RECV_PORT: {}
		VEHICLE: {}
		MAVPROXY_CMD_STR: {}
		ARDUPILOT_EXE: {}
		LAUNCH_FILE: {}
		LAUNCH_FILE_PACKAGE: {}
		SIM_MANAGER_SCRIPT: {}	
		SIM_MANAGER_PACKAGE: {}
		CONTROLLER_SCRIPT: {}
		CONTROLLER_SCRIPT_PACKAGE: {}
		""".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT, VEHICLE, MAVPROXY_CMD_STR, ARDUPILOT_EXE, LAUNCH_FILE, LAUNCH_FILE_PACKAGE, SIM_MANAGER_SCRIPT, SIM_MANAGER_PACKAGE, CONTROLLER_SCRIPT, CONTROLLER_SCRIPT_PACKAGE))

### End Configuration ###


### Start software that does not get reset on morphological change ###
# Start ROS
roscore = subprocess.Popen('roscore')
time.sleep(1)

# If debugging open transporter in a seperate terminal window
#	Otherwise open is as a subprocess
if args.debug:
	cmd_str = "bash -c \"source ~/.bashrc; rosrun evo_ros transporter.py -ip '{}' -sp {} -rp {}\"".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT)
	os.system("gnome-terminal --title 'Transporter' -e '{}'&".format(cmd_str))
else:
	cmd_str = "rosrun evo_ros transporter.py -ip '{}' -sp {} -rp {}".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT)
	transporter = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)

	

# Set up ROS subscribers and publishers
rospy.init_node('software_manager',anonymous=False)
software_ready_pub = rospy.Publisher('software_ready', std_msgs.msg.Empty, queue_size=10)
sim_result_sub = rospy.Subscriber('evaluation_instance_result', std_msgs.msg.Float64, sim_result_callback)
sim_result_pub = rospy.Publisher('evaluation_result', std_msgs.msg.Float64MultiArray, queue_size=1)

# Option for randomly selecting which launch file is used
#	Still under development!
if args.multiple_worlds:
	print("Multiple worlds option selected")
	sim_start_sub = rospy.Subscriber('received_genome', std_msgs.msg.Empty, received_genome_multiple_world_eval_callback)
else:
	sim_start_sub = rospy.Subscriber('received_genome', std_msgs.msg.Empty, received_genome_callback)


rospy.on_shutdown(shutdown_hook)
print('Everything is set up waiting for data from GA')
rospy.spin()


