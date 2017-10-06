"""
Sonar positioning GA
GAS 2017-10-03

This GA will evolve the number of sonars on the rover as well as their positions.
It will make use of a controller on the rover that is capable of handling a variable number of sonars (upto 10).

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
			'num_of_sensors':0,
			'physical':[
				# Variable number of sonar sensors (1 - 10)
				# 	Can modify x and y position within bounds of rover frame
				#	Can modify the z coordinate of orient to can direction that sonar is facing
				#{'sensor':'sonar', 'pos':[0,0,0.17], 'orient':[0,0,0]}
			],
			'behavioral':[
			]
			},
		'fitness':-1.0,
		'generation':0
		}

# Bounds for evolvable variables in the genome		
genome_constraints = {
	'physical': {
		'sonar': {
			# Position x:  0 = Middle of rover,  2.5 = front
			# Position y:  -1.5 = Left,    1.5 = Right
			'pos': {'x':[0,2.5], 'y':[-1.5,1.5]},
			# Orient z: -90 degrees = facing left,    90 = facing right
			'orient': {'z':[-90,90]}
		}
	}
} 

# Creates a thread which is responsible for sending out of individuals on the push socket
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

# The definition of the GA that will be used
class GA(object):
	def __init__(self):
		global ind
		
		self.pop_size = POP_SIZE
		self.genomes = []
		
		# Seed the initial population with a rover with a single forward facing sonar
		i = 0
		new_ind = copy.deepcopy(ind)
		new_ind['id'] = i
		new_ind['generation'] = CURRENT_GEN
		new_ind['genome']['num_of_sensors'] = 1
		new_sonar = {'sensor':'sonar1', 'pos':[0.25,0,0.17], 'orient':[0,-14,0]}
		new_ind['genome']['physical'].append(new_sonar)
		self.genomes.append(new_ind)
		
		
		# Seed the initial population with a rover that has two sonars on the front corners facing slightly outward
		i +=1
		new_ind = copy.deepcopy(ind)
		new_ind['id'] = i
		new_ind['generation'] = CURRENT_GEN
		new_ind['genome']['num_of_sensors'] = 2
		new_sonar = {'sensor':'sonar1', 'pos':[0.25,-0.1,0.17], 'orient':[0,-14,-20]}
		new_ind['genome']['physical'].append(new_sonar)
		new_sonar = {'sensor':'sonar2', 'pos':[0.25,0.1,0.17], 'orient':[0,-14,20]}
		new_ind['genome']['physical'].append(new_sonar)
		self.genomes.append(new_ind)
		i += 1
		
		#Initialize rest of population with random genomes
		while (i < self.pop_size):
			new_ind = copy.deepcopy(ind)
			new_ind['id'] = i
			new_ind['generation'] = CURRENT_GEN
			
			#Add between 1 and 10 sonars positioned randomly
			number_of_sonars = random.randint(1,10)
			new_ind['genome']['num_of_sensors'] = number_of_sonars
			
			j = 1
			while (j <= number_of_sonars):
				
				name_of_sensor = 'sonar' + str(j)
				new_sonar = {'sensor':name_of_sensor, 'pos':[0,0,0.17], 'orient':[0,-14,0]}
				
				# Pick new position (3 decimal places) and orient (int degrees)
				new_sonar['pos'][0] = round(random.uniform(genome_constraints['physical']['sonar']['pos']['x'][0], genome_constraints['physical']['sonar']['pos']['x'][1]), 3)
				new_sonar['pos'][1] = round(random.uniform(genome_constraints['physical']['sonar']['pos']['y'][0], genome_constraints['physical']['sonar']['pos']['y'][1]), 3)
				new_sonar['orient'][2]  = round(random.randint(genome_constraints['physical']['sonar']['orient']['z'][0], genome_constraints['physical']['sonar']['orient']['z'][1]))
				
				new_ind['genome']['physical'].append(new_sonar)
				j += 1
				
			self.genomes.append(new_ind)
			i += 1
		
		self.id_map = {k:v for k,v in zip([x['id'] for x in self.genomes],[i for i in range(self.pop_size)])}
		self.elite_ind = -1
		self.child_id = self.pop_size
		
		
	# Fitness is based off of 3 factors
	#	1 - Percent of course that the rover was able to complete
	#	2 - A bonus is applied if the rover was able to complete full course based off of the % allowed time remaining
	#	3 - A cost is applied corresponding to how many sonars are on the rover
	def calculate_fitness(self, return_data):		
		max_fit = 0
				
		for rd in return_data:

			temp1 = rd['fitness']
			
			
			# Calc fitness score based off of how far the rover made it in the maze
			temp = math.pow(( float(rd['fitness'][1]) / 100  + 1),2)
			
			
			# if rover finishes the maze give it a time related bonus
			if rd['fitness'][0] >= 0:
				temp = temp + math.pow((rd['fitness'][0] + 1),2)

			# Factor in the cost of sonars on the rover
			
			cost_amount = temp * 0.03 * self.genomes[self.id_map[rd['id']]]['genome']['num_of_sensors']
			
			print('Fitness before cost: {}'.format(temp))
			
			temp -= cost_amount
			
			print('Number of sensors: {}'.format(self.genomes[self.id_map[rd['id']]]['genome']['num_of_sensors']))
			print('Cost of sensors: {}'.format(cost_amount))
			
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


# Set up arg parser
parser = argparse.ArgumentParser(description='Test set up for a GA to use the rover simulation framework. Creates genomes and sends them over a TCP socket, then waits for responses from evaluation workers')
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal')
parser.add_argument('-c', '--config', type=str, help='The configuration file that is to be used')
parser.add_argument('-ip', type=str, help='Overwrite the IP address to be used to send and receive info on')
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

# Allow for IP address to be provided at command line
if args.ip is not None:
	GA_IP_ADDR = args.ip
else:
	GA_IP_ADDR = cfg['ga_server']['GA_IP_ADDR']  # If left blank, will default to IP of the machine that the script is runnng on

# If IP ADDR is blank set it to the IP of the machine this script is being ran on
if GA_IP_ADDR == '':
	#Get the IP address of the machine running this script
	str_host_IP = subprocess.check_output('hostname -I',stderr=subprocess.STDOUT,shell=True).rstrip()
	GA_IP_ADDR = str_host_IP
print('GA Host IP = {}\n'.format(GA_IP_ADDR))

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
	
	j = len(genomes)
	while j > 0:
		data = json.loads(receiver.recv())
		return_data.append({'id':data['id'], 'fitness':data['fitness']})
		j -= 1
		print('{}/{} genomes recv\'d. Result: {} \n\t Tested on: {}'.format(len(genomes) - j, len(genomes), data['fitness'], data['ns']))

	
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

