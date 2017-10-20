import random
import datetime
import copy

random.seed(datetime.datetime.now())

num_traits = 6


def sonar_random_value_mutation(population, mutation_prob, genome_constraints):
	for ind in population:
		if random.random() < mutation_prob:
			
			#print('\nBefore \n \t {}'.format(ind))
			#print('Mutation happening')
			
			# Mutation for multiple sensors
			if ind['genome']['num_of_sensors'] > 1:
				# For now just going to mutation a value on one of the sensors
				split = random.randrange(1,4,1)
				sensor_num = random.randrange(0,ind['genome']['num_of_sensors'],1)
				
				# Change X	location
				if split == 1:
					ind['genome']['physical'][sensor_num]['pos'][0] = round(random.uniform(genome_constraints['physical']['sonar']['pos']['x'][0], genome_constraints['physical']['sonar']['pos']['x'][1]), 3)
				
				# Change Y location
				if split == 2:
					ind['genome']['physical'][sensor_num]['pos'][1] = round(random.uniform(genome_constraints['physical']['sonar']['pos']['y'][0], genome_constraints['physical']['sonar']['pos']['y'][1]), 3)
				
				# Change Z Orient
				if split == 3:
					ind['genome']['physical'][sensor_num]['orient'][2] = round(random.randint(genome_constraints['physical']['sonar']['orient']['z'][0], genome_constraints['physical']['sonar']['orient']['z'][1]))
			
				
			# Mutation for a single sensor
			else:
				split = random.randrange(1,4,1)
				
				# Change X	location
				if split == 1:
					ind['genome']['physical'][0]['pos'][0] = round(random.uniform(genome_constraints['physical']['sonar']['pos']['x'][0], genome_constraints['physical']['sonar']['pos']['x'][1]), 3)
				
				# Change Y location
				if split == 2:
					ind['genome']['physical'][0]['pos'][1] = round(random.uniform(genome_constraints['physical']['sonar']['pos']['y'][0], genome_constraints['physical']['sonar']['pos']['y'][1]), 3)
				
				# Change Z Orient
				if split == 3:
					ind['genome']['physical'][0]['orient'][2] = round(random.randint(genome_constraints['physical']['sonar']['orient']['z'][0], genome_constraints['physical']['sonar']['orient']['z'][1]))
			
			ind['fitness'] = -1
			#print('\nAfter \n \t {}'.format(ind))
		
	return population

def sonar_single_point_crossover(population, cross_over_prob):
	#print('\nEntering Crossover\n')
	#print('pop size: {}'.format(len(population)))
	#print('Cross over prob: {}'.format(cross_over_prob))
	for i in range(int(len(population)/2)):
		couple = random.sample(population,2)
		if random.random() < cross_over_prob:
			
			#print('Cross over happening!')
			# If more than one sensor split between sensors
			# Still need to test this!
			min_number_of_sensors = min(couple[0]['genome']['num_of_sensors'], couple[1]['genome']['num_of_sensors'])
			
			#print('Min_number_of_sensors: {}'.format(min_number_of_sensors))
			
			child1 = copy.deepcopy(couple[0])
			child2 = copy.deepcopy(couple[1])
			
			# if more than 1 sensor on each individual do cross over at a sensor level
			if min_number_of_sensors > 1:
				split = random.randrange(1,min_number_of_sensors,1)
				#print('Split location: {}'.format(split))
				new_genome = couple[0]['genome']['physical'][0:split] + couple[1]['genome']['physical'][-1*(couple[1]['genome']['num_of_sensors']-split):]
				new_genome2 = couple[1]['genome']['physical'][0:split] + couple[0]['genome']['physical'][-1*(couple[0]['genome']['num_of_sensors']-split):]
				
				#print('Before: \n \t {}'.format(couple[0]['genome']['physical']))
				#print('\n \t {}'.format(couple[1]['genome']['physical']))
				
				#print('\n\nAfter \n \t {}'.format(new_genome))
				#print('\n \t {}'.format(new_genome2))
				
				child1['genome']['physical'] = new_genome
				child2['genome']['physical'] = new_genome2
				
			# If just one sensor on one or more of the individuals. Cross over parameters of the first sensor
			else:
				# get a number 1 - 3
				# if 1 - switch x locations
				# if 2 - switch y locations
				# if 3 - switch z orients
				split = random.randrange(1,4,1)
				
				#print('Before: \n \t {}'.format(couple[0]['genome']['physical']))
				#print('\n \t {}'.format(couple[1]['genome']['physical']))
				
				
				
				if split == 1:
					new_value = couple[0]['genome']['physical'][0]['pos'][0]
					new_value2 = couple[1]['genome']['physical'][0]['pos'][0]
					child1['genome']['physical'][0]['pos'][0] = new_value2
					child2['genome']['physical'][0]['pos'][0] = new_value
				if split == 2:
					new_value = couple[0]['genome']['physical'][0]['pos'][1]
					new_value2 = couple[1]['genome']['physical'][0]['pos'][1]
					child1['genome']['physical'][0]['pos'][1] = new_value2
					child2['genome']['physical'][0]['pos'][1] = new_value
				if split == 3:
					new_value = couple[0]['genome']['physical'][0]['orient'][2]
					new_value2 = couple[1]['genome']['physical'][0]['orient'][2]
					child1['genome']['physical'][0]['orient'][2] = new_value2
					child2['genome']['physical'][0]['orient'][2] = new_value
				
				#print('After: \n \t {}'.format(couple[0]['genome']['physical']))
				#print('\n \t {}'.format(couple[1]['genome']['physical']))
			
			# After cross over clear the fitness of the new individuals
			child1['fitness'] = -1
			child2['fitness'] = -1
			
			# Add the children into the population
			population.append(child1)
			population.append(child2)
	return population

# Overwrites the parents with the new children
def sonar_single_point_crossover_overwrite(population, cross_over_prob):
	#print('\nEntering Crossover\n')
	#print('pop size: {}'.format(len(population)))
	for i in range(int(len(population)/2)):
		couple = random.sample(population,2)
		if random.random() < cross_over_prob:
			
			#print('Cross over happening!')
			# If more than one sensor split between sensors
			# Still need to test this!
			min_number_of_sensors = min(couple[0]['genome']['num_of_sensors'], couple[1]['genome']['num_of_sensors'])
			
			#print('Min_number_of_sensors: {}'.format(min_number_of_sensors))
			
			
			# if more than 1 sensor on each individual do cross over at a sensor level
			if min_number_of_sensors > 1:
				split = random.randrange(1,min_number_of_sensors,1)
				#print('Split location: {}'.format(split))
				new_genome = couple[0]['genome']['physical'][0:split] + couple[1]['genome']['physical'][-1*(couple[1]['genome']['num_of_sensors']-split):]
				new_genome2 = couple[1]['genome']['physical'][0:split] + couple[0]['genome']['physical'][-1*(couple[0]['genome']['num_of_sensors']-split):]
				
				#print('Before: \n \t {}'.format(couple[0]['genome']['physical']))
				#print('\n \t {}'.format(couple[1]['genome']['physical']))
				
				#print('\n\nAfter \n \t {}'.format(new_genome))
				#print('\n \t {}'.format(new_genome2))
				
				couple[0]['genome']['physical'] = new_genome
				couple[1]['genome']['physical'] = new_genome2
				
			# If just one sensor on one or more of the individuals. Cross over parameters of the first sensor
			else:
				# get a number 1 - 3
				# if 1 - switch x locations
				# if 2 - switch y locations
				# if 3 - switch z orients
				split = random.randrange(1,4,1)
				
				#print('Before: \n \t {}'.format(couple[0]['genome']['physical']))
				#print('\n \t {}'.format(couple[1]['genome']['physical']))
				
				if split == 1:
					new_value = couple[0]['genome']['physical'][0]['pos'][0]
					new_value2 = couple[1]['genome']['physical'][0]['pos'][0]
					couple[0]['genome']['physical'][0]['pos'][0] = new_value2
					couple[1]['genome']['physical'][0]['pos'][0] = new_value
				if split == 2:
					new_value = couple[0]['genome']['physical'][0]['pos'][1]
					new_value2 = couple[1]['genome']['physical'][0]['pos'][1]
					couple[0]['genome']['physical'][0]['pos'][1] = new_value2
					couple[1]['genome']['physical'][0]['pos'][1] = new_value
				if split == 3:
					new_value = couple[0]['genome']['physical'][0]['orient'][2]
					new_value2 = couple[1]['genome']['physical'][0]['orient'][2]
					couple[0]['genome']['physical'][0]['orient'][2] = new_value2
					couple[1]['genome']['physical'][0]['orient'][2] = new_value
				
				#print('After: \n \t {}'.format(couple[0]['genome']['physical']))
				#print('\n \t {}'.format(couple[1]['genome']['physical']))
			
			# After cross over clear the fitness of the new individuals
			couple[0]['fitness'] = -1
			couple[1]['fitness'] = -1
	return population

def single_point_crossover(population, cross_over_prob):
	for i in range(len(population)):
		couple = random.sample(population,2)
		

		if random.random() < cross_over_prob:
			#pick a random spot for single point cross over
			split = random.randrange(0,num_traits,1)
			new_genome = couple[0]['genome']['behavioral'][0:split] + couple[1]['genome']['behavioral'][-1*(num_traits-split):]
			new_genome2 = couple[1]['genome']['behavioral'][0:split] + couple[0]['genome']['behavioral'][-1*(num_traits-split):]
			
			couple[0]['genome']['behavioral'] = new_genome
			couple[1]['genome']['behavioral'] = new_genome2
	
	return population

def random_value_mutation(population, mutation_prob):
	for ind in population:
		if random.random() < mutation_prob:
			#Select a gene that is to be mutated
			gene_num = random.randrange(0, num_traits,1)

			if gene_num == 0:
				ind['genome']['behavioral'][0]['max_turn_strength'] = random.randrange(50,400,1)
			if gene_num == 1:
				ind['genome']['behavioral'][1]['max_yaw_change_per_cb'] = random.randrange(1,100,1)
			if gene_num == 2:
				ind['genome']['behavioral'][2]['num_vision_cones'] = random.randrange(1,101,2)
			if gene_num == 3:
				ind['genome']['behavioral'][3]['sweep_weight_factor'] = random.random()*5
			if gene_num == 4:
				ind['genome']['behavioral'][4]['distance_weight_factor'] = random.random()*5
			if gene_num == 5:
				ind['genome']['behavioral'][5]['wall_distance'] = random.random()*10
				
	return population


def format_print(ind):
	print(ind['id'])
	for gene in ind['genome']['behavioral']:
		print(gene)
	return
	

#Not finished
def convert_genome_to_binary(ind):
	print('Entering binary conversion')
	format_print(ind)
	
	chromosome = 0
	for gene in ind['genome']['behavioral']:
		#print(gene.keys())
		#print(gene.values())
		print('gene: {} \t value:{} \t binary:{}'.format(gene.keys(),gene.values(),bin(gene.values()[0])[2:]))
	
	return
		
	
