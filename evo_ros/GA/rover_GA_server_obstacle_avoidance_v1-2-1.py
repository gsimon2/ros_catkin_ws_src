"""
    Simple non-ga server for the rover.
"""
 
import json
import zmq
import argparse
import random
import threading
import subprocess
import os
import datetime
import copy 
import sys
import time
import math
import yaml

from GA_operators import random_value_mutation
from GA_operators import single_point_crossover


# Individual structure and default values
ind = {'id':0,
		'genome':{
			'physical':[
				{'sensor':'lidar', 'pos':[0,0,0.4], 'orient':[0,0,0]}
			],
			'behavioral':[
				{'max_turn_strength':200}, #int, [50-400]
				{'max_yaw_change_per_cb':15}, #int, [0-100]
				{'num_vision_cones':7}, #int, [1-101], must be odd
				{'sweep_weight_factor':1},#float, [0-5]
				{'distance_weight_factor':1},#float, [0-5]
				{'wall_distance':3} #float, [0-10]
			]
			},
		'fitness':-1.0,
		'generation':0
		}


class SenderThread(threading.Thread):
	def __init__(self, threadID, socket, genomes):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.socket = socket
		self.data = genomes
		 
	def run(self):
		#print("Gnome - max_turn_strength, max_yaw_change_per_cb, num_vision_cones, sweep_weight_factor, distance_weight_factor")
		#print("\t\t\t\tStarting Sender Thread:"+str(self.threadID))
		self.send_data()
		#print("\t\t\t\tExiting Sender Thread:"+str(self.threadID))
 
	def send_data(self):
		""" Send data to worker processes. 
		
		Args:
			socket: socket to send the data out on.
				- Persistant throughout execution for now.
		"""
        
		for i in self.data:
			msg = json.dumps(i)
			self.socket.send(msg)

	def send_tear_down_msg(self):
		
		for i in range(NUM_EVAL_WORKS):
			ind['id'] = -1
			print('Sending finish msg')
			msg = json.dumps(ind)
			self.socket.send(msg)


class GACommunicator(object):
	""" Class to handle setting up the sockets and sending/receiving genome data. """

	def __init__(self):
		#Initialize the socket for data
		
		# Setup the socket to send data out on.
		context = zmq.Context()
		self.socket = context.socket(zmq.PUSH)
		#socket.setsockopt(zmq.LINGER, 0)    # discard unsent messages on close
		self.socket.bind('tcp://{}:{}'.format(GA_IP_ADDR, GA_SEND_PORT))
		 
		# Setup the socket to read the responses on.
		self.receiver = context.socket(zmq.PULL)
		self.receiver.bind('tcp://{}:{}'.format(GA_IP_ADDR, GA_RECV_PORT))
		
	def __del__(self):
		print("Closing Sockets")
		self.socket.close()
		self.receiver.close()
	
	def send_tear_down_msg(self):
		sendThread = SenderThread(1, self.socket, '')
		sendThread.send_tear_down_msg()
		
	def send_genomes(self,genomes):
		""" Send the genomes through the sender thread to workers. 
		Args:
			genomes: list of genomes to send out.
		Returns:
			list of results containing the genome id and fitness
		"""

		return_data = []
		
		# Start a thread to send the data.
		sendThread = SenderThread(1, self.socket, genomes)
		
		sendThread.start()
		
		# Read the responses on the receiver socket.
		i = len(genomes)
		while i > 0:
			data = json.loads(self.receiver.recv())
			return_data.append({'id':data['id'], 'fitness':data['fitness']})
			i -= 1
         
		# Wait for the send thread to complete.
		sendThread.join()
		
		return return_data



class GA(object):
	def __init__(self):
		global ind
		
		self.pop_size = POP_SIZE
		
		#Initialize a population with random genomes
		self.genomes = []
		for i in range(self.pop_size):
			new_ind = copy.deepcopy(ind)
			new_ind['id'] = i
			new_ind['generation'] = CURRENT_GEN
			new_ind['genome']['behavioral'][0]['max_turn_strength'] = random.randrange(50,400,1)
			new_ind['genome']['behavioral'][1]['max_yaw_change_per_cb'] = random.randrange(1,100,1)
			new_ind['genome']['behavioral'][2]['num_vision_cones'] = random.randrange(1,101,2)
			new_ind['genome']['behavioral'][3]['sweep_weight_factor'] = random.random()*5
			new_ind['genome']['behavioral'][4]['distance_weight_factor'] = random.random()*5
			new_ind['genome']['behavioral'][5]['wall_distance'] = random.random()*10
			self.genomes.append(new_ind)
		
		self.id_map = {k:v for k,v in zip([x['id'] for x in self.genomes],[i for i in range(self.pop_size)])}
		self.elite_ind = -1
		#self.ga_communicator = GACommunicator()
		self.child_id = self.pop_size
		
	def tear_down(self):
		self.ga_communicator.send_tear_down_msg()
		
	def calculate_fitness(self, return_data):
		#return_data = self.ga_communicator.send_genomes(self.genomes)
		
		max_fit = 0
				
		#print(self.id_map)
		#print(return_data)
		#print(self.genomes)
		
		for rd in return_data:

			temp1 = rd['fitness']
			
			#Calc fitness score based off of how far the rover made it in the maze
			temp = math.pow(( float(rd['fitness'][1]) / 100  + 1),2)
			
			#if rover finishes the maze give it a time related bonus
			if rd['fitness'][0] >= 0:
				temp = temp + math.pow((rd['fitness'][0] + 1),2)

			rd['fitness'] = temp
			
			print('returned result: {} \t Calc fitness: {}'.format(temp1, rd['fitness']))

			self.genomes[self.id_map[rd['id']]]['fitness'] = rd['fitness']
			
			if rd['fitness'] > max_fit:
				max_fit = rd['fitness']
				self.elite_ind = copy.deepcopy(self.genomes[self.id_map[rd['id']]])
		
		
		#print('Returned data: {}'.format(return_data))
		print('\n\n Winning Ind for generation {}:\n {}\n'.format(CURRENT_GEN,self.elite_ind))
		#print(max_fit)
		return
		
	def next_generation(self):
		""" Modify the population for the next generation. """
		child_pop = [copy.deepcopy(self.elite_ind)]
		
		# Perform tournament selection.
		for i in range(len(self.genomes)-1):
			tourn = random.sample(self.genomes,TOURNAMENT_SIZE)
						
			fitness_list = []
			for ind in tourn:
				fitness_list.append(ind['fitness'])
			
			winner_index = fitness_list.index(max(fitness_list))			
			child_pop.append(copy.deepcopy(tourn[winner_index]))
					
			child_pop[-1]['id'] = self.child_id
			self.child_id += 1

		
		#Crossover
		child_pop = single_point_crossover(child_pop, CROSS_OVER_PROB)

		# Mutate genes in the child genomes.
		child_pop = random_value_mutation(child_pop, MUTATION_PROB)
		
		for child in child_pop:
			child['generation'] = CURRENT_GEN + 1

		self.genomes = child_pop
		self.id_map = {k:v for k,v in zip([x['id'] for x in self.genomes],[i for i in range(self.pop_size)])}
		
	def ga_log(self, log):
		global CURRENT_GEN
		for ind in self.genomes:
			log.write('{}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(CURRENT_GEN, ind['id'], ind['genome']['behavioral'][0]['max_turn_strength'], ind['genome']['behavioral'][1]['max_yaw_change_per_cb'], ind['genome']['behavioral'][2]['num_vision_cones'], ind['genome']['behavioral'][3]['sweep_weight_factor'], ind['genome']['behavioral'][4]['distance_weight_factor'], ind['genome']['behavioral'][5]['wall_distance'], ind['fitness']))
		
	def get_pop(self):
		return self.genomes



def resend(genomes, socket, return_data):
	print('Resending!')
	
	#load any genomes that have not had results collected from them into a list
	uncollected_ind = []
	for ind in genomes:
		if not any(return_ind['id'] == ind['id'] for return_ind in return_data):
			uncollected_ind.append(ind)
	
	print('Uncollected indvidiuals:')
	for ind in uncollected_ind:
		print(ind['id'])
	
	# Load resend genomes that have not had results collected for them yet
	sendThread = SenderThread(1, socket, uncollected_ind)
	sendThread.start()
	sendThread.join()

# Set up arg parser
parser = argparse.ArgumentParser(description='Test set up for a GA to use the rover simulation framework. Creates genomes and sends them over a TCP socket, then waits for responses from evaluation workers')
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal')
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

# Seed rand num generator	
rand_seed = datetime.datetime.now()
random.seed(rand_seed)

# Load in values from config file
NUM_EVAL_WORKS = cfg['ga_server']['NUM_EVAL_WORKS']
GA_IP_ADDR = cfg['ga_server']['GA_IP_ADDR']  # If left blank, will default to IP of the machine that the script is runnng on
GA_SEND_PORT = cfg['ga_server']['GA_SEND_PORT']
GA_RECV_PORT = cfg['ga_server']['GA_RECV_PORT']
LOG_FILE_NAME = cfg['ga_server']['LOG_FILE_NAME'] #Log file name
MAX_WAIT_TIME = cfg['ga_server']['MAX_WAIT_TIME'] # Max wait time on evaluation collection socket before we resend genomes in miliseconds
POP_SIZE = cfg['ga_server']['POP_SIZE'] # How large the population size is for each generation
GEN_COUNT = cfg['ga_server']['GEN_COUNT'] # How many generations is this experiment going to run for
CURRENT_GEN = 0 # Reports the current generation - Always starts at 0
MUTATION_PROB = cfg['ga_server']['MUTATION_PROB'] #Probability that an individual will have a random gene mutated
CROSS_OVER_PROB = cfg['ga_server']['CROSS_OVER_PROB'] #Probability that two individuals will cross over and producing mixed offspring
TOURNAMENT_SIZE = cfg['ga_server']['TOURNAMENT_SIZE'] # Number of individuals that enter each selection tournament to create the next generation


# If IP ADDR is blank set it to the IP of the machine this script is being ran on
if GA_IP_ADDR == '':
	#Get the IP address of the machine running this script
	str_host_IP = subprocess.check_output('hostname -I',stderr=subprocess.STDOUT,shell=True).rstrip()
	print('Found Host IP = {}\n'.format(str_host_IP))
	GA_IP_ADDR = str_host_IP
	
if args.debug:
	print('Debugging option has been turned on!\n')

# If debugging print configuration settings to screen
if args.debug:
	print("""\n\tConfiguration Settings...
		NUM_EVAL_WORKS: {}
		GA_IP_ADDR: {}
		GA_SEND_PORT: {}
		GA_RECV_PORT: {}
		LOG_FILE_NAME: {}
		MAX_WAIT_TIME: {}
		POP_SIZE: {}
		GEN_COUNT: {}
		MUTATION_PROB: {}
		CROSS_OVER_PROB: {}	
		TOURNAMENT_SIZE: {}
		""".format(NUM_EVAL_WORKS, GA_IP_ADDR, GA_SEND_PORT, GA_RECV_PORT, LOG_FILE_NAME, MAX_WAIT_TIME, POP_SIZE, GEN_COUNT, MUTATION_PROB, CROSS_OVER_PROB,TOURNAMENT_SIZE))

### End Configuration ###



 
#Create a log file
log = open('logs/{}'.format(LOG_FILE_NAME), 'w+')

#Write experiment parameters to log
log.write('*****Rover GA expirement*****\n')
log.write('Date:{}\n'.format(datetime.datetime.now()))
log.write('Population size:{}\n'.format(POP_SIZE))
log.write('Generation Count:{}\n'.format(GEN_COUNT))
log.write('Mutation Probability:{}\n'.format(MUTATION_PROB))
log.write('Cross Over Probability:{}\n'.format(CROSS_OVER_PROB))
log.write('*****************************\n')
log.write('Generation, ID, Max Turn Strength, Max Yaw Change per CB, Num Vision Cones, Sweep Weight Factor, Distance Weight Factor, Wall Distance, Fitness\n')




#Initialize the socket for data

# Setup the socket to send data out on.
context = zmq.Context()
socket = context.socket(zmq.PUSH)
#socket.setsockopt(zmq.LINGER, 0)    # discard unsent messages on close
socket.bind('tcp://{}:{}'.format(GA_IP_ADDR, GA_SEND_PORT))
 
# Setup the socket to read the responses on.
receiver = context.socket(zmq.PULL)
receiver.bind('tcp://{}:{}'.format(GA_IP_ADDR, GA_RECV_PORT))

# Setup ZMQ poller
poller = zmq.Poller()
poller.register(receiver, zmq.POLLIN)

		
print("Press Enter when the workers are ready: ")
_ = raw_input()
print("Sending tasks to workers")

start_time = datetime.datetime.now()
ga= GA()

for i in range(GEN_COUNT):
	CURRENT_GEN = i
	
	genomes = ga.get_pop()
	
	#print('Sending out IDs:')
	#for ind in genomes:
	#	print('\t {}'.format(ind['id']))
	return_data = []
	
	# Start a thread to send the data.
	sendThread = SenderThread(1, socket, genomes)
	
	sendThread.start()
	
	# Wait for the send thread to complete.
	sendThread.join()
	
	num_unevaluated_ind = len(genomes)
	while num_unevaluated_ind > 0:
		try:
			print('Waiting for data')
			socks = dict(poller.poll(MAX_WAIT_TIME))
			if socks:
				if socks.get(receiver) == zmq.POLLIN:
					data = json.loads(receiver.recv(zmq.NOBLOCK))
			else:
				print('Timeout on receiver socket occured!')
				resend(genomes, socket, return_data)
				continue
			
			# Check if data for the recv'd ind is already in returned data
			#	This can happen if we have to resend out data mid generation
			if any(return_ind['id'] == data['id'] for return_ind in return_data):
				print('Recv\'d multiple results for ID: {}'.format(data['id']))
			
			# New data so store it in the returned data
			else:
				#check to make sure collected result is from this generation
				if any(ind['id'] == data['id'] for ind in genomes):
					return_data.append({'id':data['id'], 'fitness':data['fitness']})
					num_unevaluated_ind -= 1
					print('{}/{} genomes recv\'d. ID: {} Result: {} \n\t Tested on: {}'.format(len(genomes) - num_unevaluated_ind, len(genomes), data['id'], data['fitness'], data['ns']))
				else:
					print('Recv\'d result from previous generation')
				
		#if results are not being collected send out any individuals from generation that have not been evaluated yet
		except KeyboardInterrupt:
			user_input = raw_input("\nEnter 'r' to resend uncollected individuals or enter anything else to quit: ")
			if user_input is 'r':
				resend(genomes, socket, return_data)
			else:
				#Tear down evo-ros framework
				#sendThread = SenderThread(1, socket, '')
				#sendThread.send_tear_down_msg()
				#sendThread.start()
				#sendThread.join()
				sys.exit()
	
	# Calcuate fitnes for this generation, log it, and prepare the next generation
	ga.calculate_fitness(return_data)
	ga.ga_log(log)
	ga.next_generation()
	time.sleep(3)
	
#Tear down evo-ros framework
sendThread = SenderThread(1, socket, '')
sendThread.send_tear_down_msg()
sendThread.start()
sendThread.join()

end_time = datetime.datetime.now()
running_time = end_time - start_time
print('Start time: {}\n End time: {}\n Running time: {}\n'.format(start_time,end_time,running_time))

