"""
Fixed Number Sonar positioning GA
GAS 2018-10-20

Evolve positioning of a fixed number of sonars on the rover

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

from GA_operators import sonar_random_value_mutation
from GA_operators import sonar_single_point_crossover

MAX_NUMBER_OF_SONAR = 2

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
		'raw_fitness':[],
		'generation':0
		}

# Bounds for evolvable variables in the genome		
genome_constraints = {
	'physical': {
		'sonar': {
			# Position x:  0 = Middle of rover,  0.25 = front
			# Position y:  -0.15 = Left,    0.15 = Right
			'pos': {'x':[0,0.25], 'y':[-0.15,0.15]},
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
		self.child_pop = []
		
		i = 0
		
		# Seed the initial population with a rover with a single forward facing sonar
		"""
		new_ind = copy.deepcopy(ind)
		new_ind['id'] = i
		new_ind['generation'] = CURRENT_GEN
		new_ind['genome']['num_of_sensors'] = 1
		new_sonar = {'sensor':'sonar1', 'pos':[0.25,0,0.17], 'orient':[0,-14,0]}
		new_ind['genome']['physical'].append(new_sonar)
		self.genomes.append(new_ind)
		i +=1
		"""
		
		# Seed the initial population with a rover that has two sonars on the front corners facing slightly outward
		
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
			#number_of_sonars = random.randint(1,MAX_NUMBER_OF_SONAR)
			number_of_sonars = MAX_NUMBER_OF_SONAR
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
		self.child_id_map = {k:v for k,v in zip([x['id'] for x in self.genomes],[i for i in range(self.pop_size)])}
		self.elite_ind = -1
		self.child_id = self.pop_size
		self.child_pop = copy.deepcopy(self.genomes)
		self.genomes = []
		
	# Fitness is based off of 3 factors
	#	1 - Percent of course that the rover was able to complete
	#	2 - A bonus is applied if the rover was able to complete full course based off of the % allowed time remaining
	#	3 - A cost is applied corresponding to how many sonars are on the rover
	def calculate_fitness(self, return_data):		
		max_fit = 0
				
		for rd in return_data:

			raw_fitness = rd['fitness']
			rd['fitness'] = 0
			
			i = 0
			while i < len(raw_fitness):
				# Calc fitness score based off of how far the rover made it in the maze
				calc_fitness = math.pow(( float(raw_fitness[i+1]) / 100  + 1),2)
			
			
				# if rover finishes the maze give it a time related bonus
				if raw_fitness[i] >= 0:
					calc_fitness = calc_fitness + math.pow((raw_fitness[i] + 1),2)
			
				#print('\t ID: {} \t Partial fitness: {} \t I: {}'.format(rd['id'], calc_fitness, i))
				rd['fitness'] += calc_fitness
				i += 2
			
			
			#print('returned result: {} \t Calc fitness: {}'.format(raw_fitness, rd['fitness']))

			self.child_pop[self.child_id_map[rd['id']]]['fitness'] = rd['fitness']
			self.child_pop[self.child_id_map[rd['id']]]['raw_fitness'] = raw_fitness
			
			if rd['fitness'] > max_fit:
				max_fit = rd['fitness']
				self.elite_ind = copy.deepcopy(self.child_pop[self.child_id_map[rd['id']]])
		
		print('\n\n Winning Ind for generation {}:\n {}\n'.format(CURRENT_GEN,self.elite_ind))
		return
		
		
	# performs cross over and mutation to create children from the current population
	def create_children_population(self):
		self.child_pop = []
		population_pool = copy.deepcopy(self.genomes)
		
		#Crossover
		population_pool = sonar_single_point_crossover(population_pool, CROSS_OVER_PROB)

		# Mutate genes in the population pool.
		population_pool = sonar_random_value_mutation(population_pool, MUTATION_PROB, genome_constraints)
		
		# Filter out any children or mutated individuals by looking for a fitness of -1
		child_pop = []
		for ind in population_pool:
			if ind['fitness'] == -1:
				ind['id'] = self.child_id
				self.child_id += 1
				self.child_pop.append(ind)
		
		print('Size of child pop: {}'.format(len(self.child_pop)))
		self.child_id_map = {k:v for k,v in zip([x['id'] for x in self.child_pop],[i for i in range(len(self.child_pop))])}
		print('Child mapping: {}'.format(self.child_id_map))
		#return self.child_pop
		
		
		
	def next_generation(self):
		population_pool = self.genomes + self.child_pop
		next_generation = []
		next_generation.append(copy.deepcopy(self.elite_ind))
		
		# Perform tournament selection.
		for i in range(self.pop_size-1):
			tourn = random.sample(population_pool,TOURNAMENT_SIZE)
						
			fitness_list = []
			for ind in tourn:
				fitness_list.append(ind['fitness'])
			
			winner_index = fitness_list.index(max(fitness_list))			
			next_generation.append(copy.deepcopy(tourn[winner_index]))
		
		self.genomes = next_generation
		
		
	def ga_log(self, LOG_FILE_NAME):
		global CURRENT_GEN
		
		# The initial population is only stored in the child_pop
		if CURRENT_GEN == 0:
			population = self.child_pop
		else:
			population = self.genomes
			
		log = open('logs/{}'.format(LOG_FILE_NAME), 'a')
		for ind in population:
			log.write('{}, {}, {}, {}, '.format(CURRENT_GEN, ind['id'], ind['fitness'], ind['genome']['num_of_sensors']))
			
			for sensor in ind['genome']['physical']:
				log.write('{}, {}, {}, '.format(sensor['pos'][0], sensor['pos'][1], sensor['orient'][2]))
				
			
			for element in ind['raw_fitness']:
				log.write('{}, '.format(element))
			log.write('\n')
		log.close()
		
	def get_pop(self):
		return self.genomes
	
	def get_unevaluated_pop(self):
		unevaluated_pop = []
		for ind in self.genomes:
			#print(ind)
			if ind['fitness'] <= 0:
				unevaluated_pop.append(ind)
		return unevaluated_pop
	
		

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
parser.add_argument('-ip', type=str, help='Overwrite the IP address to be used to send and receive info on')
parser.add_argument('-log', type=str, help='Name of the logfile to write to')
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
rand_seed = random.randrange(1,19,1)
random.seed(rand_seed)

# Load in values from config file
NUM_EVAL_WORKS = cfg['ga_server']['NUM_EVAL_WORKS']
GA_SEND_PORT = cfg['ga_server']['GA_SEND_PORT']
GA_RECV_PORT = cfg['ga_server']['GA_RECV_PORT']
MAX_WAIT_TIME = cfg['ga_server']['MAX_WAIT_TIME'] # Max wait time on evaluation collection socket before we resend genomes in miliseconds
POP_SIZE = cfg['ga_server']['POP_SIZE'] # How large the population size is for each generation
GEN_COUNT = cfg['ga_server']['GEN_COUNT'] # How many generations is this experiment going to run for
CURRENT_GEN = 0 # Reports the current generation - Always starts at 0
MUTATION_PROB = cfg['ga_server']['MUTATION_PROB'] #Probability that an individual will have a random gene mutated
CROSS_OVER_PROB = cfg['ga_server']['CROSS_OVER_PROB'] #Probability that two individuals will cross over and producing mixed offspring
TOURNAMENT_SIZE = cfg['ga_server']['TOURNAMENT_SIZE'] # Number of individuals that enter each selection tournament to create the next generation

	
# Allow for log file name to be provided at command line
if args.log is not None:
	LOG_FILE_NAME = args.log
else:
	LOG_FILE_NAME = cfg['ga_server']['LOG_FILE_NAME'] #Log file name

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
log.write('*****Sonar Placement GA Expirement*****\n')
log.write('Start Time: {}\n'.format(datetime.datetime.now()))
log.write('Rand_seed:{}\n'.format(rand_seed))
log.write('End Time: \n')
log.write('Running Time: \n')
log.write('Population size:{}\n'.format(POP_SIZE))
log.write('Generation Count:{}\n'.format(GEN_COUNT))
log.write('Mutation Probability:{}\n'.format(MUTATION_PROB))
log.write('Cross Over Probability:{}\n'.format(CROSS_OVER_PROB))
log.write('***** Constraints *****\n')
log.write('Max Number of Sensors: {}\n'.format(MAX_NUMBER_OF_SONAR))
log.write('X Position: {}\n'.format(str(genome_constraints['physical']['sonar']['pos']['x'])))
log.write('Y Position: {}\n'.format(str(genome_constraints['physical']['sonar']['pos']['y'])))
log.write('Z Orient: {}\n'.format(str(genome_constraints['physical']['sonar']['orient']['z'])))
log.write('*****************************\n')
log.write('Generation, ID, Fitness, Number of Sonar, ')

for i in range(1,MAX_NUMBER_OF_SONAR+1):
	pos_x = 'S{}_P_X'.format(i)
	pos_y = 'S{}_P_Y'.format(i)
	ori_z = 'S{}_O'.format(i)
	log.write('{}, {}, {}, '.format(pos_x, pos_y, ori_z))
log.write('Raw Fitness')
log.write('\n')
log.close()

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
	
	#genomes = ga.get_pop()
	genomes = ga.child_pop
	print('\n\nSending new generation!!!')
	print('{} individuals need to be evaluated'.format(len(genomes)))
	
	#print('Sending out IDs:')
	#for ind in genomes:
	#	print('\t {}'.format(ind['id']))
	return_data = []
	
	# Start a thread to send the data.
	sendThread = SenderThread(1, socket, genomes)
	
	sendThread.start()
	
	# Wait for the send thread to complete.
	sendThread.join()
	
	
	if len(genomes) > 0 :
		j = len(genomes)
		while j > 0:
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
						j -= 1
						print('{}/{} genomes recv\'d. ID: {} Result: {} \n\t Tested on: {}'.format(len(genomes) - j, len(genomes), data['id'], data['fitness'], data['ns']))
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
	ga.ga_log(LOG_FILE_NAME)
	ga.next_generation()
	ga.create_children_population()
	time.sleep(1)
	
#Tear down evo-ros framework
sendThread = SenderThread(1, socket, '')
sendThread.send_tear_down_msg()
sendThread.start()
sendThread.join()

# Close file so that we can edit the end time and running time in the header
log.close()
end_time = datetime.datetime.now()
running_time = end_time - start_time
with open('logs/{}'.format(LOG_FILE_NAME), 'r') as file:
    data = file.readlines()
data[3] = data[3].strip() + ' ' + str(end_time) + '\n' 
data[4] = data[4].strip() + ' ' + str(running_time) + '\n' 
with open('logs/{}'.format(LOG_FILE_NAME), 'w') as file:
    file.writelines( data )

print('Start time: {}\n End time: {}\n Running time: {}\n'.format(start_time,end_time,running_time))

