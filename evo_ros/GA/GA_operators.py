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
		
{'id':0,
		'genome':{
			'num_of_sensors':1,
			'physical':[
				# Variable number of sonar sensors (1 - 10)
				# 	Can modify x and y position within bounds of rover frame
				#	Can modify the z coordinate of orient to can direction that sonar is facing
				#{'sensor':'sonar', 'pos':[0,0,0.17], 'orient':[0,0,0]}
			],
			'behavioral':[
			],
			'position_encoding':[
                            {'sensor':'sonar1', 'region':2, 'pos':[99,99,0.17],'orient': [0,0,0]}]
			},
		'fitness':-1.0,
		'raw_fitness':[],
		'generation':0
		}
        
        

  
def region_calc(region,x,y,z):
    new_x =0
    new_y =0
    new_z =z
    x =  float(x)/ 100.0
    y = float(y)/100.0
    if region ==3:
        new_x = round(0.0 + x*.20,3)
        new_y = round(-.15 +y*.05,3)
    if region ==2:
        new_x = round(.2 + x* .05,3)
        new_y = round(-.10+y*.20,3)
    if region ==1:
        new_x = round(0.0 +x*.2,3)
        new_y = round(.10 +y*.05,3)
    if region ==5:
        new_x = round(.20 +x *.05,3)
        new_y = round( -.15+y*.05,3)
    if region ==4:
        new_x = round(.20+x*.05,3)
        new_y = round(.10+y*.05,3)
    return [new_x,new_y,new_z]

def pos_from_region(population):
    for ind in population:
        encoding = ind['genome']['position_encoding']
        ind['genome']['physical'] = []
        counter =0
        ind['genome']['num_of_sensors'] = len(encoding)
        for sensor in encoding:
            physical_sensor = {'sensor': 'sonar'+str(counter+1),'pos': region_calc(sensor['region'],sensor['pos'][0],sensor['pos'][1],sensor['pos'][2]), 'orient' : sensor['orient'] }
            ind['genome']['physical'].append(copy.deepcopy(physical_sensor))
            counter+=1
            
genome_constraints = {
	'physical': {
		'sonar': {
			# Position x:  0 = Middle of rover,  0.25 = front
			# Position y:  -0.15 = Left,    0.15 = Right
			'pos': {'x':[0,100], 'y':[0,100], 'z': .17},
            'regions': [1,5],
            'region_orient': {1: [-90,-40],2:[-30,30],3:[40,90],4:[-100,0],5:[0,100]},
			# Orient z: -90 degrees = facing left,    90 = facing right
			'orient': {'z':[-90,90]}
            }
        }
        }


def delete_mirrored_sonars(population):
	for ind in population:
		#a = [item for item in a if ...]
		ind['genome']['position_encoding'] = [sensor for sensor in ind['genome']['position_encoding'] if sensor['type'] == 'original']
		ind['genome']['num_of_sensors'] /= 2
	pos_from_region(population)
	return population
				
			
			
def create_mirrored_sonars(population):
	for ind in population:
		for sensor in ind['genome']['position_encoding']:
			
			# Create a copy of the sensor that we are mirroring
			new_sensor = copy.deepcopy(sensor)
			new_sensor['type'] = 'mirrored'
			
			# Add one to the count of sensors on this individual
			ind['genome']['num_of_sensors'] += 1
			
			# Change the name of the new sensor
			new_sensor['sensor'] = 'sensor' + str(ind['genome']['num_of_sensors'])
			
			# Mirror the new sensor
			if sensor['type'] == 'original':
				
				# Switch the region that it is in, unless in it is the front middle region
				if sensor['region'] == 1:
					new_sensor['region'] = 3
				elif sensor['region'] == 3:
					new_sensor['region'] = 1
				elif sensor['region'] == 4:
					new_sensor['region'] = 5
				elif sensor['region'] == 5:
					new_sensor['region'] = 4
				
				# Front middle region stays in the same region
				elif sensor['region'] == 2:
					pass
				else:
					print('Error unknown region found in create_mirrored_sonars function!')
					exit()
				
				# Flip the sonar angle
				new_sensor['orient'][2] = sensor['orient'][2] * -1
				# Flip the y positioning
				new_sensor['pos'][1] = 100 - sensor['pos'][1]
				
				# Add the sensor to the list
				ind['genome']['position_encoding'].append(new_sensor)
				
			else:
				pass
	pos_from_region(population)
	return population
		
		
		
def generate_pop(pop_size,max_sensors,constraints):
    start_ind = {'id':0,
    		'genome':{
    			'num_of_sensors':1,
    			'physical':[
    				# Variable number of sonar sensors (1 - 10)
    				# 	Can modify x and y position within bounds of rover frame
    				#	Can modify the z coordinate of orient to can direction that sonar is facing
    				#{'sensor':'sonar', 'pos':[0,0,0.17], 'orient':[0,0,0]}
    			],
    			'behavioral':[
    			],
    			'position_encoding':[
					{'sensor':'sonar1', 'region':2, 'pos':[.99,.99,0.17],'orient': [0,0,0]}]
    			},
    		'fitness':-1.0,
    		'raw_fitness':[],
    		'generation':0
    }
        
    population = list()
    for number in range(pop_size):
        new_ind = copy.deepcopy(start_ind)
        new_ind['genome']['position_encoding'] =list()
        new_ind['id'] = number
        number_sensors = random.randint(1,max_sensors)
        new_ind['genome']['num_of_sensors'] = number_sensors
        for sensor in range(number_sensors):
            region = random.randint(constraints['physical']['sonar']['regions'][0],constraints['physical']['sonar']['regions'][1])
            x = random.randint(constraints['physical']['sonar']['pos']['x'][0],constraints['physical']['sonar']['pos']['x'][1])
            y = random.randint(constraints['physical']['sonar']['pos']['y'][0],constraints['physical']['sonar']['pos']['y'][1])
            z = constraints['physical']['sonar']['pos']['z']
            orient = random.randint(constraints['physical']['sonar']['region_orient'][region][0],constraints['physical']['sonar']['region_orient'][region][1]) * -1
            new_sensor = {'sensor':'sonar'+str(sensor+1),'region':region,'pos':[x,y,z], 'orient': [0,-14,orient]}
            new_ind['genome']['position_encoding'].append(new_sensor)
        population.append(new_ind)
    pos_from_region(population)
    return population

def add_remove_random_mutation(population, mutation_prob, constraints,max_sensors):
    for ind in population:
        if random.random() < mutation_prob:
            mut_option = random.randrange(1,7,1)
            if(mut_option==1):
                #change x
                sensor_to_change = random.randrange(0,ind['genome']['num_of_sensors'],1)
                ind['genome']['position_encoding'][sensor_to_change]['pos'][0] = random.randint(constraints['physical']['sonar']['pos']['x'][0],constraints['physical']['sonar']['pos']['x'][1])
            elif(mut_option==2):
                #change y
                sensor_to_change = random.randrange(0,ind['genome']['num_of_sensors'],1)
                ind['genome']['position_encoding'][sensor_to_change]['pos'][1] = random.randint(constraints['physical']['sonar']['pos']['y'][0],constraints['physical']['sonar']['pos']['y'][1])
            elif(mut_option==3):
                #change orient
                sensor_to_change = random.randrange(0,ind['genome']['num_of_sensors'],1)
                region = ind['genome']['position_encoding'][sensor_to_change]['region']
                ind['genome']['position_encoding'][sensor_to_change]['orient'][2] = random.randint(constraints['physical']['sonar']['region_orient'][region][0],constraints['physical']['sonar']['region_orient'][region][1])
            elif(mut_option==4):
                #change region and fix orient
                sensor_to_change = random.randrange(0,ind['genome']['num_of_sensors'],1)
                ind['genome']['position_encoding'][sensor_to_change]['region'] = random.randint(constraints['physical']['sonar']['regions'][0],constraints['physical']['sonar']['regions'][1])
                region = ind['genome']['position_encoding'][sensor_to_change]['region']
                ind['genome']['position_encoding'][sensor_to_change]['orient'][2] = random.randint(constraints['physical']['sonar']['region_orient'][region][0],constraints['physical']['sonar']['region_orient'][region][1])
            elif(mut_option==5):
                # add a sensor
                num_sensors = ind['genome']['num_of_sensors']
                if(num_sensors <max_sensors):
                    #if not maxed on sensors add otherwise delete
                    region = random.randint(constraints['physical']['sonar']['regions'][0],constraints['physical']['sonar']['regions'][1])
                    x = random.randint(constraints['physical']['sonar']['pos']['x'][0],constraints['physical']['sonar']['pos']['x'][1])
                    y = random.randint(constraints['physical']['sonar']['pos']['y'][0],constraints['physical']['sonar']['pos']['y'][1])
                    z = constraints['physical']['sonar']['pos']['z']
                    orient = random.randint(constraints['physical']['sonar']['region_orient'][region][0],constraints['physical']['sonar']['region_orient'][region][1])
                    new_sensor = {'sensor':'sonar'+str(num_sensors+1),'region':region,'pos':[x,y,z], 'orient': [0,-14,orient]}
                    ind['genome']['position_encoding'].append(new_sensor)
                    ind['genome']['num_of_sensors'] = ind['genome']['num_of_sensors'] +1
                else:
                    #otherwise delete one
                    sensor_to_change = random.randrange(0,ind['genome']['num_of_sensors'],1)
                    ind['genome']['position_encoding'].pop(sensor_to_change)
                    ind['genome']['num_of_sensors'] = ind['genome']['num_of_sensors'] -1
            elif(mut_option==6):
                #delete snesor otherwise add
                num_sensors = ind['genome']['num_of_sensors']
                if(num_sensors >1):
                    sensor_to_change = random.randrange(0,ind['genome']['num_of_sensors'],1)
                    ind['genome']['position_encoding'].pop(sensor_to_change)
                    ind['genome']['num_of_sensors'] = ind['genome']['num_of_sensors'] -1
                else:
                    #add sensor
                    region = random.randint(constraints['physical']['sonar']['regions'][0],constraints['physical']['sonar']['regions'][1])
                    x = random.randint(constraints['physical']['sonar']['pos']['x'][0],constraints['physical']['sonar']['pos']['x'][1])
                    y = random.randint(constraints['physical']['sonar']['pos']['y'][0],constraints['physical']['sonar']['pos']['y'][1])
                    z = constraints['physical']['sonar']['pos']['z']
                    orient = random.randint(constraints['physical']['sonar']['region_orient'][region][0],constraints['physical']['sonar']['region_orient'][region][1])
                    new_sensor = {'sensor':'sonar'+str(num_sensors),'region':region,'pos':[x,y,z], 'orient': [0,-14,orient]}
                    ind['genome']['position_encoding'].append(new_sensor)
                    ind['genome']['num_of_sensors'] = ind['genome']['num_of_sensors'] +1
                
                
			
            ind['fitness'] = -1
            #print('\nAfter \n \t {}'.format(ind))
		
    return population
# copy of crossover for  CSE848 project
def multiple_sensor_crossover(population, cross_over_prob, max_sensors,constraints):
    for i in range(int(len(population)/2)):
        couple = random.sample(population,2)
        
        if random.random() < cross_over_prob:
            child1 = copy.deepcopy(couple[0])
            child2 = copy.deepcopy(couple[1])
            min_number_of_sensors = min(couple[0]['genome']['num_of_sensors'], couple[1]['genome']['num_of_sensors'])
           # if(min_number_of_sensors >1):
            if True:
                #print('GAHHHHHH')
                #print('child 1:{}\t{} \n\n '.format(child1['genome']['num_of_sensors'],child1['genome']['physical']))
                #print('child 1 Encoding: {} \n '.format(child1['genome']['position_encoding']))
                #print('child 2: {}\t{}\n '.format(child2['genome']['num_of_sensors'],child2['genome']['physical']))
                #print('child 2 Encoding: {}\n '.format(child2['genome']['position_encoding']))
                
                if min_number_of_sensors == 1:
                    split = 1
                else:
                   split = random.randrange(1,min_number_of_sensors,1)
                
                #print('split at: {}'.format(split))
                encoding1 = copy.deepcopy(couple[0]['genome']['position_encoding'][:split]) + copy.deepcopy(couple[1]['genome']['position_encoding'][split:])
                encoding2 = copy.deepcopy(couple[1]['genome']['position_encoding'][:split]) + copy.deepcopy(couple[0]['genome']['position_encoding'][split:])
                child1['genome']['position_encoding'] = copy.deepcopy(encoding1)
                child1['genome']['num_of_sensors'] = len(child1['genome']['position_encoding'])
                child2['genome']['position_encoding'] = copy.deepcopy(encoding2)
                child2['genome']['num_of_sensors'] = len(child2['genome']['position_encoding'])
                
                pos_from_region([child1])
                pos_from_region([child2])
                #print('child 1:{}\t{} \n\n '.format(child1['genome']['num_of_sensors'],child1['genome']['physical']))
                #print('child 1 Encoding: {} \n '.format(child1['genome']['position_encoding']))
                #print('child 2: {}\t{}\n '.format(child2['genome']['num_of_sensors'],child2['genome']['physical']))
                #print('child 2 Encoding: {}\n n\n\n'.format(child2['genome']['position_encoding']))
                
                
            else:
                print('This is happening with more than one sensor')
                print('child 1: \n {}'.format(child1))
                print('child 2: \n {}'.format(child2))
                #one of the rovers has 1 sensor
                sensor1 = random.randrange(0,len(couple[0]['genome']['position_encoding']),1)
                sensor2 = random.randrange(0,len(couple[0]['genome']['position_encoding']),1)
                cross_num = random.randrange(1,4,1)
                
                print('Sensor 1: {}'.format(sensor1))
                print('Sensor 2: {}'.format(sensor2))
                print('Cross num: {}'.format(cross_num))
                if(cross_num==1):
                    #swap x
                    temp =copy.deepcopy(child1['genome']['position_encoding'][sensor1]['pos'][0])
                    child1['genome']['position_encoding'][sensor1]['pos'][0] = copy.deepcopy(child2['genome']['position_encoding'][sensor2]['pos'][0])
                    child2['genome']['position_encoding'][sensor2]['pos'][0] = temp
                elif(cross_num==2):
                    #swap y
                    temp =copy.deepcopy(child1['genome']['position_encoding'][sensor1]['pos'][1])
                    child1['genome']['position_encoding'][sensor1]['pos'][1] = copy.deepcopy(child2['genome']['position_encoding'][sensor2]['pos'][1])
                    child2['genome']['position_encoding'][sensor2]['pos'][1] = temp
                    pass
                elif(cross_num==3):
                    #swap region and orient
                    temp_region =copy.deepcopy(child1['genome']['position_encoding'][sensor1]['region'])
                    temp_orient =copy.deepcopy(child1['genome']['position_encoding'][sensor1]['orient'])
                    child1['genome']['position_encoding'][sensor1]['region'] = copy.deepcopy(child2['genome']['position_encoding'][sensor2]['region'])
                    child1['genome']['position_encoding'][sensor1]['orient'] = copy.deepcopy(child2['genome']['position_encoding'][sensor2]['orient'])
                    child2['genome']['position_encoding'][sensor2]['region']= temp_region
                    child2['genome']['position_encoding'][sensor2]['orient'] = temp_orient
                    pass
                
            child1['fitness'] = -1
            child2['fitness'] = -1
            population.append(child1)
            population.append(child2)
    return population

                
