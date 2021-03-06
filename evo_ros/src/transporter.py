#!/usr/bin/env python 
#
# Transporter
#
# Connects to the GA's TCP socket and receives genome packets. Creates a
# ROS transporter node that is responsible for loading the genome into a 
# ros parameter and sending out the start_sim message on the start_sim topic.
# It then waits for the simulation results and relays them over a TCP 
# socket to the GA.
#
# GAS 2017-7-18

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
import yaml

evaluation_result = []

GA_SEND_PORT = 5000
GA_RECV_PORT = 5010
GA_IP_ADDR = '127.0.0.1'

# Max wait time on evaluation collection socket before we resend genomes
MAX_WAIT_TIME = 5 * 60 * 1000 # In miliseconds

### Callback function for the simulation_result topic ###
### 	Receives data and loads it into the global evaluation_ressult var ###
###		to be sent back to the GA over a TCP socket not managed by ROS ###
def sim_result_callback(data):
    global evaluation_result
    evaluation_result = data.data


### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""Connects to the GA's TCP socket and receives genome packets.
	Creates a ROS transporter node that is responsible for loading the genome into a 
	ros parameter and sending out the start_sim message on the start_sim topic.
	It then waits for the simulation results and relays them over a TCP 
	socket to the GA.""")
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal')
parser.add_argument('-sp', '--ga_send_port', type=int, help='Port number that the GA is sending the genomes on')
parser.add_argument('-rp' , '--ga_recv_port', type=int, help='Port number that the GA is receiving the results on')
parser.add_argument('-ip' , '--ga_ip_addr', type=str, help='IP address that the GA is running on')
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

if args.ga_send_port is not None:
	GA_SEND_PORT = args.ga_send_port
else:
	GA_SEND_PORT = cfg['transporter']['GA_SEND_PORT']

if args.ga_recv_port is not None:
	GA_RECV_PORT = args.ga_recv_port
else:
	GA_RECV_PORT = cfg['transporter']['GA_RECV_PORT']
	
if args.ga_ip_addr is not None:
	GA_IP_ADDR = args.ga_ip_addr
else:
	GA_IP_ADDR = cfg['transporter']['GA_IP_ADDR']
	
# If debugging print configuration settings to screen
if args.debug:
	print("""\n\tConfiguration Settings...
		GA_IP_ADDR: {}
		GA_SEND_PORT: {}
		GA_RECV_PORT: {}
		""".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT))

### End Configuration ###




### Setup the contexts for communicating with the outside server. ###
recv_addr_str = 'tcp://{}:{}'.format(GA_IP_ADDR, GA_SEND_PORT)
send_addr_str = 'tcp://{}:{}'.format(GA_IP_ADDR, GA_RECV_PORT)

context = zmq.Context()
receiver = context.socket(zmq.PULL)
receiver.connect(recv_addr_str)
sender = context.socket(zmq.PUSH)
sender.connect(send_addr_str)
#print("GA is running at IP: {} \n Sending port: {} \n Receiving port: {} \n".format(GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT))

# Setup ZMQ poller
poller = zmq.Poller()
poller.register(receiver, zmq.POLLIN)



### Setup the ROS topics for communicating with connected nodes. ###
rospy.init_node('transporter',anonymous=False)
pub = rospy.Publisher('received_genome', std_msgs.msg.Empty, queue_size=5)
sub = rospy.Subscriber('evaluation_result', std_msgs.msg.Float64MultiArray, sim_result_callback)
print("Ros transport node and simulation start/result topics have been initialized")



#### Get host name - Sent back with sim results so GA knows who data is coming from ###
str_host_name = socket.gethostname()


### Loop ###
### 	Receive genome from GA, load it into a ros parameter, wait for sim_result ###
###		If receive end message from GA ('id' = -1), send end message to sim_manager and tear down ###
while True:
	
	# Receive data from GA
	print('Waiting for data from GA...')
	
	socks = dict(poller.poll(MAX_WAIT_TIME))
	if socks:
		if socks.get(receiver) == zmq.POLLIN:
			data = json.loads(receiver.recv(zmq.NOBLOCK))
	else:
		print('Timeout on receiver socket occured!')
		continue
	
	print('Transporter: Received: {}'.format(data))
			
	# Load the data into a parameter in ROS
	rospy.set_param('vehicle_genome', data)
	
	time.sleep(1)
	
	# Send a ready message on the sim_start topic
	pub.publish(std_msgs.msg.Empty())
	
	print('Genome data loaded into ROS param and sim start message sent! Entering sleep until sim evaluation is complete.')
	
	# Wait for simulation result
	while evaluation_result == []:
		time.sleep(0.5)
	
	# Get percent complete from rospy params and prep the simulation result msg for the GA
	#percent_complete = rospy.get_param('percent_complete')
	msg = json.dumps({'id':data['id'],'fitness':evaluation_result, 'ns':str_host_name, 'name':rospy.get_name()})
	
	# Send simulation result msg
	sender.send(msg)
	print ("""Sent result for ID: {}\n 
		Evaluation Result: {}""".format(data['id'], evaluation_result))
	evaluation_result = []
	

### Tear down transporter ###
print("Exiting...")
	
	


