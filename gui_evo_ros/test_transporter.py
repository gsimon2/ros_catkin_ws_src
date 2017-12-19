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

evaluation_result = ''

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
#rospy.init_node('transporter',anonymous=False)
#pub = rospy.Publisher('received_genome', std_msgs.msg.Empty, queue_size=5)
#sub = rospy.Subscriber('evaluation_result', std_msgs.msg.Float64, sim_result_callback)
#print("Ros transport node and simulation start/result topics have been initialized")



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

	
	# Wait for simulation result
	#_ = raw_input()
	print('Sening result...')
	
	# Get percent complete from rospy params and prep the simulation result msg for the GA
	
	return_fitness = []
	for i in range(2):
		return_fitness.append(random.random())
		
	msg = json.dumps({'id':data['id'],'fitness':return_fitness, 'ns':str_host_name, 'name':rospy.get_name()})
	
	# Send simulation result msg
	sender.send(msg)
	evaluation_result = ''
	

### Tear down transporter ###
print("Exiting...")
	
	


