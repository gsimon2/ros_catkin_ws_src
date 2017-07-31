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

import std_msgs.msg

from add_sensor_functions import add_lidar
from add_sensor_functions import copy_base_rover_file
from argparse import RawTextHelpFormatter


last_physical_genome = []
recv_first_msg = False

evaluation_result = ''

random.seed(datetime.datetime.now())

# Default controller script
# TO-DO change this to contorller_script
CONTROLLER_SCRIPT = 'rover_controller.py'

# Default launch file to be used
# TO-DO modify launch files to reflect seperate changes
LAUNCH_FILE = 'maze_1.launch'

# Default GUI state for running Gazebo
#	Note: Headless and GUI should always be opposite of each other
HEADLESS = 'true'
GUI = 'false'

# The current generation of genomes that is being evaluated
GENERATION = 0

# IP address that external GA is running at
GA_IP_ADDR = '127.0.0.1'

# The port number that the GA is sending genomes out on
GA_SEND_PORT = 5000

# The port number that the GA is collecting results on
GA_RECV_PORT = 5010



### Simulation Result Callback ###
### 	Collect received sim result data and store it in global var
def sim_result_callback(data):
	global last_physical_genome
	print('Evaluation instance completed with result: {}'.format(data.data))
	# Check when the simulation takes too long to complete and reset the software
	#	For the next received genome
	if data.data == -2:
		print('Reseting stored genome due to sim timeout')
		last_physical_genome = []
	

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
	cmd_str = "pkill -1 -f sim_manager.py"
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
	cmd_str = "killall -9 gzserver gzclient mavproxy.py xterm"
	os.system(cmd_str)
	if args.debug is False:
		try:
			mavproxy.kill()
			launch_file.kill()
			rover_behavior.kill()
			mavproxy.wait()
			launch_file.wait()
			rover_behavior.wait()
		except Exception:
			pass
	time.sleep(3)

	# Get host name of machine #
	str_host_name = socket.gethostname()

	#Create a copy of the base rover file for this instance
	str_rover_file = copy_base_rover_file(str_host_name)
	
	#Build rover sensors based off recveived genome
	for genome_trait in data['genome']['physical']:
		print('{}\n'.format(genome_trait))
		if 'lidar' in genome_trait['sensor']:
			print("Adding a lidar sensor to the rover")
			#Add sensors based off genome
			add_lidar(str_rover_file, genome_trait['pos'], genome_trait['orient'])

	time.sleep(1)
	
	### Start all needed processes ###
	# Start MAVProxy
	if args.debug:
		cmd_str = """xterm -title 'MAVProxy' -hold  -e '
			source ~/simulation/ros_catkin_ws/devel/setup.bash;
			cd ~/simulation/ardupilot/APMrover2;
			echo \"param load ~/simulation/ardupilot/Tools/Frame_params/3DR_Rover.param\";
			echo
			echo \" (For manual control) - param set SYSID_MYGCS 255\";
			echo
			echo \" (For script control) - param set SYSID_MYGCS 1\";
			../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo'&"""
		os.system(cmd_str)
	else:
		cmd_str = """source ~/simulation/ros_catkin_ws/devel/setup.bash;
			cd ~/simulation/ardupilot/APMrover2;
			echo \"param load ~/simulation/ardupilot/Tools/Frame_params/3DR_Rover.param\";
			echo
			echo \" (For manual control) - param set SYSID_MYGCS 255\";
			echo
			echo \" (For script control) - param set SYSID_MYGCS 1\";
			../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo"""
		mavproxy = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)
	
	#Give time to start up Mavproxy and Ardupilot (takes a while since Ardupilots sim_vehicle script calls xterm to start the ardupilot scripts)
	str_PID = ''
	while(str_PID == ''):
		try:
			str_PID = subprocess.check_output('pidof APMrover2.elf',stderr=subprocess.STDOUT,shell=True)
		except Exception:
			pass
		time.sleep(0.5)
	print('Started MAVProxy!')
	time.sleep(4)
	
	# Run launch file
	if args.debug:
		cmd_str = "xterm -hold -e 'roslaunch rover_ga {} model:={} gui:={} headless:={}'&".format(LAUNCH_FILE, str_rover_file, GUI, HEADLESS)
		os.system(cmd_str)
		time.sleep(1)
	else:
		cmd_str = 'roslaunch rover_ga {} model:={} gui:={} headless:={}'.format(LAUNCH_FILE, str_rover_file, GUI, HEADLESS)
		launch_file = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)
	print('Started launch file!')
	
	# Start Sim manager node
	cmd_str = "rosrun evo-ros sim_manager.py"
	if args.debug:
		os.system("xterm -hold -e '{}'&".format(cmd_str))
	else:
		sim_manager = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)
		
	#Start controller script
	if args.debug:
		cmd_str = "xterm -hold -e 'rosrun evo-ros {}'&".format(CONTROLLER_SCRIPT)
		os.system(cmd_str)
	else:
		cmd_str = 'rosrun evo-ros {}'.format(CONTROLLER_SCRIPT)
		rover_behavior = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)

	#Give time for everything to start up
	if args.less_wait:
		time.sleep(10)
	else:
		if args.debug:
			time.sleep(45) #spawning a bunch of xterms for debugging takes longer than subprocesses
		else:
			time.sleep(30)
	
	
### Simulation Start Callback ###
###
def sim_start_callback(recv_data):
	#Get genome data
	data = rospy.get_param('rover_genome')
	print('Received genome data!')
	
	# Check for ending msg
	if data['id'] == -1:
		print('Received exit message')
		rospy.signal_shutdown('Received exit message from GA')
		return
	
	# Update Generation
	if GENERATION != data['generation']:
		global GENERATION
		GENERATION = data['generation']
		
		# If want to spawn vehicle in a different world for each generation
		#	change the launch file being used when we receive a new gen
		if args.multiple_worlds:
			last_physical_genome = []
			global LAUNCH_FILE
			LAUNCH_FILE = pick_new_launch()
	print('Current generation: {}'.format(data['generation']))
	
	#Check to see if received physical genome is different from last received
	if data['genome']['physical'] != last_physical_genome:
		print("		Received different physical genome!")
		global last_physical_genome
		last_physical_genome = data['genome']['physical']
		software_setup(data)
	else:
		print("	 	Same pyhsical genome!")

	# Notify the sim_manager that all evaluation software is set up
	software_ready_pub.publish(std_msgs.msg.Empty())
	
	print('Done! Entering sleep onto sim evaluation is complete.')
	

### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""
Responsible for two main functions:
	-Spawning all required software for evaluating the genome
		(Gazebo, other ROS nodes, and Ardupilot if needed)
	-Reading the physical (morphology) aspect of the genome and modifying
		the vehicle .urdf file as needed
	""", formatter_class=RawTextHelpFormatter)
parser.add_argument('-cs' , '--controller_script', type=str, help='The controller script to be used for the vehicle')
parser.add_argument('-lf' , '--launch_file', type=str, help='The launch file that is to be used')
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
parser.add_argument('-mw' , '--multiple_worlds', action='store_true', help='Tells software_manager to choose new launch files between generations \n New launch files must be specified in the pick_new_launch function')
parser.add_argument('-gui', '--graphics', action='store_true', help='Start gazebo gui for each simulation')
parser.add_argument('--less_wait',action='store_true',help='Minimize the sleep timers to make running on local machines faster. This option will cause problems when running on remote VMs')
parser.add_argument('-sp', '--ga_send_port', type=int, help='Port number that the GA is sending the genomes on')
parser.add_argument('-rp' , '--ga_recv_port', type=int, help='Port number that the GA is receiving the results on')
parser.add_argument('-ip' , '--ga_ip_addr', type=str, help='IP address that the GA is running on')

args= parser.parse_args()



if args.controller_script is not None:
	CONTROLLER_SCRIPT = args.controller_script
	
if args.launch_file is not None:
	LAUNCH_FILE = args.launch_file

if args.multiple_worlds:
	print("Multiple worlds option selected")
	global LAUNCH_FILE
	LAUNCH_FILE = pick_new_launch()
	
if args.graphics:
	print('Turning on Gazebo GUI')
	HEADLESS = 'false'
	GUI = 'true'
	
if args.ga_send_port is not None:
	GA_SEND_PORT = args.ga_send_port

if args.ga_recv_port is not None:
	GA_RECV_PORT = args.ga_recv_port
	
if args.ga_ip_addr is not None:
	GA_IP_ADDR = args.ga_ip_addr



### Start software that does not get reset on morphological change ###
# Start ROS
roscore = subprocess.Popen('roscore')
time.sleep(1)

# If debugging open transporter in a seperate terminal window
#	Otherwise open is as a subprocess
if args.debug:
	cmd_str = "bash -c \"source ~/.bashrc; rosrun evo-ros transporter.py -ip '{}' -sp {} -rp {}\"".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT)
	os.system("gnome-terminal --title 'Transporter' -e '{}'&".format(cmd_str))
else:
	cmd_str = "rosrun evo-ros transporter.py -ip '{}' -sp {} -rp {}".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT)
	transporter = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)

	

# Set up ROS subscribers and publishers
rospy.init_node('software_manager',anonymous=False)
software_ready_pub = rospy.Publisher('software_ready', std_msgs.msg.Empty, queue_size=1)
sim_result_sub = rospy.Subscriber('simulation_result', std_msgs.msg.Float64, sim_result_callback)
sim_start_sub = rospy.Subscriber('simulation_start', std_msgs.msg.Empty, sim_start_callback)


rospy.on_shutdown(shutdown_hook)
rospy.spin()


