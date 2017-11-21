# GAS 2017-08-14
import rospy
import time

import cv2
import numpy as np
import math
import mavros

from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import OverrideRCIn

import numpy as np
import std_msgs.msg

#How dratastically the rover will try to turn
# 0 = no turning
# 400 = max turning strength
max_turn_strength = 200

#how much the yaw can change for each callback
max_yaw_change_per_cb = 10

#distance at which a rover will try to stop or reverse instead of going
#	around an object
min_detection_distance = 0.45

#Number of how many partitions there will be of the lidar sweep
#	this must be an odd number
num_vision_cones = 7

#Modifies the weighting of objects as they appear further to the side 
#	of the rover
sweep_weight_factor = 1

#Modifies the weighting of the objects as they come closer to the rover
distance_weight_factor = 1

#How close the rover gets to a wall before it turns at max strength to try to avoid it
wall_distance = 1

last_nav_cmd = {'throttle':1900,'yaw':1500}


def findMiddle(input_list):
    middle = float(len(input_list))/2
    if middle % 2 != 0:
        return input_list[int(middle - .5)]
    else:
        return (input_list[int(middle)], input_list[int(middle-1)])

### Parse Genome ###
### 	Parses the received genome and assigns any obstacle avoidance
###		related traits to script variables
def parse_genome(genome):
	
	global max_turn_strength
	global max_yaw_change_per_cb
	global min_detection_distance
	global num_vision_cones
	global sweep_weight_factor
	global distance_weight_factor
	
	for genome_trait in genome['behavioral']:
		#print('{}\n'.format(genome_trait))
		if 'max_turn_strength' in genome_trait:
			max_turn_strength = genome_trait['max_turn_strength']
		if 'max_yaw_change_per_cb' in genome_trait:
			max_yaw_change_per_cb = genome_trait['max_yaw_change_per_cb']
		if 'num_vision_cones' in genome_trait:
			num_vision_cones = genome_trait['num_vision_cones']
		if 'sweep_weight_factor' in genome_trait:
			sweep_weight_factor = genome_trait['sweep_weight_factor']
		if 'distance_weight_factor' in genome_trait:
			distance_weight_factor = genome_trait['distance_weight_factor']
		if 'wall_distance' in genome_trait:
			wall_distance = genome_trait['wall_distance']
		

	print("""Gnome - max_turn_strength {}, \n
	max_yaw_change_per_cb {}, \n
	num_vision_cones {}, \n
	sweep_weight_factor {}, \n
	distance_weight_factor {}, \n
	wall_distance {} \n""".format(max_turn_strength,max_yaw_change_per_cb,num_vision_cones, sweep_weight_factor,distance_weight_factor, wall_distance))

### Check Vision ###
###		Takes the partitioned vision and calculates a navigation command
###		to avoid any obstacles in the way
def check_vision(data, vision):
	global last_nav_cmd
	
	#print('partitioned_vision: {}'.format(vision))
	nav_cmds = {'throttle':1900,'yaw':1500}
	
	
	#Handle right half of vision (not counting center cone)
	# If objects are detected, want to decrease yaw to turn rover left
	for i in range(len(vision)/2):

		#weight for how steep to turn based off of where the object is
		#	detected in lidar sweep. Closer to center gets higher weight
		sweep_weight = sweep_weight_factor * (float((i+1)) / (len(vision)/2))
		
		#weight for how steep to turn based off of how close the object 
		#	is to the rover. Closer gets higher weight
		distance_weight = distance_weight_factor * float((data.range_max - vision[i]) / data.range_max)
		
		#If we don't see anything in this cone, don't change yaw at all
		if vision[i] == data.range_max:
			nav_cmds['yaw'] = nav_cmds['yaw']
		#turn left. Sharpest turn will happen with close objects closer to the front of the rover
		#	while distance objects off to the side will result in little to no turn
		else:
			nav_cmds['yaw'] = nav_cmds['yaw'] - (400 * sweep_weight * distance_weight)
		
		#print('Right side: \t i: {} \t sweep_weight: {} \n\t distance_weight: {} \t yaw: {}'.format(i,sweep_weight, distance_weight,nav_cmds['yaw']))



	#Handle left half of vision (not counting center cone)
	# If objects are detected, want to increase yaw to turn rover right
	for i in range(len(vision)/2 +1, len(vision)):
		
		#remap i to j for sweep_weight purposes
		#	Gives the outer cones less weight
		j = len(vision) - (i+1)

		#weight for how steep to turn based off of where the object is
		#	detected in lidar sweep. Closer to center gets higher weight
		sweep_weight = sweep_weight_factor * (float((j+1)) / (len(vision)/2))
		
		#weight for how steep to turn based off of how close the object 
		#	is to the rover. Closer gets higher weight
		distance_weight = distance_weight_factor * float((data.range_max - vision[i]) / data.range_max)
		
		#If we don't see anything in this cone, don't change yaw at all
		if vision[i] == data.range_max:
			nav_cmds['yaw'] = nav_cmds['yaw']
		#turn right. Sharpest turn will happen with close objects closer to the front of the rover
		#	while distance objects off to the side will result in little to no turn
		else:
			nav_cmds['yaw'] = nav_cmds['yaw'] + (400 * sweep_weight * distance_weight)
				
		#print('Left side: \t i: {} \t j: {} \t sweep_weight: {} \n\t distance_weight: {} \t yaw: {}'.format(i,j,sweep_weight, distance_weight,nav_cmds['yaw']))
	
	
	#Handle straight forward
	#
	#
	middle_index = int(len(vision) / 2)
	
	#weight for how steep to turn based off of how close the object 
	#	is to the rover. Closer gets higher weight
	distance_weight = distance_weight_factor * float((data.range_max - vision[middle_index]) / data.range_max)
	
	#If we don't see anything in this cone, don't change yaw at all
	if vision[middle_index] == data.range_max:
			nav_cmds['yaw'] = nav_cmds['yaw']
	else:
		#if rover is already turning right, turn right sharper based on how close the object is
		if nav_cmds['yaw'] >= 1500:
			nav_cmds['yaw'] = nav_cmds['yaw'] + (400 * distance_weight)
			
		#same but with if rover is already turning left
		else:
			nav_cmds['yaw'] = nav_cmds['yaw'] - (400 * distance_weight)
			
	
	#Make sure that yaw stays between [1500 - max_turn_strength, 1500 + max_turn_strength]
	if nav_cmds['yaw'] > 1500 + max_turn_strength:
		nav_cmds['yaw'] = 1500 + max_turn_strength			
	if nav_cmds['yaw'] < 1500 - max_turn_strength:
		nav_cmds['yaw'] = 1500 - max_turn_strength
	
	#smooth out jerkiness of turns by limiting how much yaw can change
	#	on each callback
	difference = nav_cmds['yaw'] - last_nav_cmd['yaw']
	if  abs(difference) > max_yaw_change_per_cb:
		if difference > 0:
			nav_cmds['yaw'] = last_nav_cmd['yaw'] + max_yaw_change_per_cb
		else:
			nav_cmds['yaw'] = last_nav_cmd['yaw'] - max_yaw_change_per_cb
		
	last_nav_cmd = nav_cmds
	
	# If an object is detected within the wall_distance, turn at max strength
	if vision[middle_index] <= wall_distance:
		nav_cmds['throttle'] = 1600
	#if rover is already turning right, turn right sharp
		if nav_cmds['yaw'] >= 1500:
			nav_cmds['yaw'] = 1500 + max_turn_strength	
		#same but with if rover is already turning left
		else:
			nav_cmds['yaw'] = 1500 - max_turn_strength
	return nav_cmds

### Partition Vision ###
###		Takes in the data from the lidar sensor and divides it into a variable
###		number of partitions each with the average range value for that section.
###		Can optionally display a visual representation of what is being seen by the lidar
def partition_vision(data, num_vision_cones = 5, show_visual = False):
	
	#Set up cv2 image to visualize the lidar readings
	frame = np.zeros((500, 500,3), np.uint8)
	
						 
	partitioned_vision = [data.range_max for x in range(num_vision_cones)] #List of the average value in each section
	
	
	#Create a list of the upper beam index value for each vision cone
	#  IE - first cone will have values indexed from 0 to (data.lenth / num_vision_cones)
	#	    Second cone will have (data.lenth / num_vision_cones) + 1 to 2 x data.lenth / num_vision_cones)
	partition_index = [(len(data.ranges)/num_vision_cones) for i in range(num_vision_cones)]
	middle_index = int(num_vision_cones / 2) #not '+1' because indexing starts at 0 for the list
	
	# Add additional ones to middle if don't match sum.
	if sum(partition_index) < len(data.ranges):
		partition_index[middle_index] += len(data.ranges) - sum(partition_index)
	for i in range(1,num_vision_cones):
		partition_index[i] = partition_index[i] + partition_index[i-1]
		
		
	#This will be used to help draw the cv2 image 
	angle = data.angle_max
	
	# Assign received range data information and save it to a new list where the 'inf'
	#	ranges are set to the max range set for the lidar sensor being used
	range_data = [0 for x in range(len(data.ranges))]
	for i in range(len(data.ranges)):
		if data.ranges[i] == float ('Inf'):
			range_data[i] = float(data.range_max)
		else:
			range_data[i] = float(data.ranges[i])
			
		#Draw lines in cv2 image to help visualize the lidar reading
		x = math.trunc( (range_data[i] * 10)*math.cos(angle + (-90*3.1416/180)) )
		y = math.trunc( (range_data[i] * 10)*math.sin(angle + (-90*3.1416/180)) )
		
		#If the line is on a vision cone boundary draw it red otherwise blue
		if i in partition_index:
			cv2.line(frame,(250, 250),(x+250,y+250),(0,0,255),1)
		else:
			cv2.line(frame,(250, 250),(x+250,y+250),(255,0,0),1)
		angle= angle - data.angle_increment
		
	
	#Get averages for get vision cone
	for i in range(num_vision_cones):
		if i == 0:
			partitioned_vision[i] = sum(range_data[0:partition_index[i]])/len(range_data[0:partition_index[i]])	
		else:
			partitioned_vision[i] = sum(range_data[partition_index[i-1]:partition_index[i]])/len(range_data[partition_index[i-1]:partition_index[i]])
	
	#display cv2 image
	if show_visual:
		cv2.imshow('frame',frame)
		cv2.waitKey(1)
	
	return partitioned_vision
	
def ErleRover_Obstacle_Avoidance(data):
	frame = np.zeros((500, 500,3), np.uint8)
	angle = data.angle_max
	Vx = 250
	Vy = 250
	for r in data.ranges:
		if r == float ('Inf'):
			r = data.range_max
		x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
		y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
		cv2.line(frame,(250, 250),(x+250,y+250),(255,0,0),1)
		Vx+=x
		Vy+=y
		angle= angle - data.angle_increment
	
	cv2.line(frame,(250, 250),(250+Vx,250+Vy),(0,0,255),3)
	cv2.circle(frame, (250, 250), 2, (255, 255, 0))
	ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
	if ang > 180:
		ang -= 360
	cv2.putText(frame,str(ang)[:10], (50,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))
	
	cv2.imshow('frame',frame)
	cv2.waitKey(1)
	
	yaw = 1500 + ang * 40 / 6
	throttle = 1900
	nav_cmds = {'throttle':throttle,'yaw':yaw}
	return nav_cmds
	


# Sonar Avoidance
#   Iterate through a variable number of sonars each one casting a vote on which direction to turn
#   Votes are weighted by how close an object is detected on that sonar
#	Votes are then used to decide course of action
#	If result is close and mutliple objects are detected, then turn sharply one direction
#
# TO-DO: Implement timer that selections a direction to turn if result is close and multiple sonar detect an object
def sonar_avoidance(sonar_ranges, sonar_angles, range_max):
	nav_cmds = {'throttle':1900,'yaw':1500}
	turn_voting = {'left':0.0, 'right':0.0}
	angle_limit = 3 # degrees from center that define which sonars are to be considered to be pointing forward
	#print('range max: {}'.format(range_max))
	#print(sonar_ranges)
	#print(sonar_angles)
	
	# Let each sonar that detects an object cast a vote if they should turn right or left
	for sonar_id in sonar_ranges:
		# Detect if object within range
		if sonar_ranges[sonar_id] < range_max:
			
			# Votes are weighted higher if the object is closer to the vehile
			vote_weight = abs(sonar_ranges[sonar_id] - range_max)
			
			#object in the middle of view
			if sonar_angles[sonar_id] >= -angle_limit and sonar_angles[sonar_id] <= angle_limit:
				# Turn one way or the other
				turn_voting['left'] += vote_weight
			
			# object on the right
			if sonar_angles[sonar_id] < -angle_limit:
				turn_voting['left'] += vote_weight
			
			# object on the left
			if sonar_angles[sonar_id] > angle_limit:
				turn_voting['right'] += vote_weight
	
	#print(turn_voting)
	
	# Calculate which way we should turn based off of the votes
	result = turn_voting['right'] - turn_voting['left']
	#print('Result: {}'.format(result))
	
	# if result is close to 0, but not 0 means that an object is most likely directly in front of vehicle
	if abs(result) <= 1 and turn_voting['left'] > 1.5 and turn_voting['right'] > 1.5:
		nav_cmds['yaw'] = 1100
		# turn sharply left or right
		
	# Handle objects on the right or left
	else:
		# Cap result max at range_max
		if result < -range_max:
			result = -range_max
		if result > range_max:
			result = range_max
		
		# Weight the turn strength based off of the magnitude of the result
		#	Higher magnitude corresponds to closer objects
		turn_weight = 1 - (range_max - abs(result)) / range_max
		
		#print('turn_weight: {}'.format(turn_weight))
		
		# If result is negative turn left
		if result < 0:
			nav_cmds['yaw'] = 1500 - 400 * turn_weight
		# If result is positive turn right
		else:
			nav_cmds['yaw'] = 1500 + 400 * turn_weight
			
	#print('nav Cmds: {}'.format(nav_cmds))
	
	return nav_cmds

# Sonar Hybrid Obstacle Avoidance / Way point Following
#	Tries to avoidance obstaces, but still takes into account the vechile heading and the bearing to the next waypoint
def sonar_hybrid_avoidance(sonar_ranges, sonar_angles, range_max, bearing, heading):
	nav_cmds = {'throttle':1900,'yaw':1500}
	
	# Get the direction that we need to turn 
	#	Positive = Right	Negative = Left
	heading_diff = bearing - heading
	
	# Cap the heading diff at the max steering angle for the rover
	#	Max steering angle found in apm_plugin_gazebo_side.cpp in the rover folder of the ardupilot/Gazebo plugin package
	max_steering_ang = 88.544 / 2 # Divide by 2 becuase They measure the ang has wheel direction left to right, we want ang from center
	if heading_diff > max_steering_ang:
		heading_diff = max_steering_ang
	if heading_diff < -1 * max_steering_ang:
		heading_diff = -1 * max_steering_ang
		
	# Find the turn ratio
	#	0 = straight	0.5 = turn right at half max strength	-1 = turn left at full max strength		etc
	turn_ratio = heading_diff / max_steering_ang
	
	
	# Scan the sonar to see if any are pointing in the direction of the heading
	for sonar_id in sonar_ranges:
		if sonar_angles[sonar_id] - 10 <= heading_diff <= sonar_angles[sonar_id] + 10:
			# Detect if object within range
			if sonar_ranges[sonar_id] < range_max:
				pass
				#print('Object between me and waypoint')
				
	# Convert the turn ratio into a yaw command to head the rover to the next waypoint
	nav_cmds['yaw'] = 1500 + ( 400 * turn_ratio)	
	
	
	
	
	return nav_cmds
	
	
# Drive at bearing
#	Directs rover to a specific bearing and drives that direction
def drive_at_bearing(bearing, heading):
	nav_cmds = {'throttle':1900,'yaw':1500}
	
	# Get the direction that we need to turn 
	#	Positive = Right	Negative = Left
	heading_diff = bearing - heading
	
	# Cap the heading diff at the max steering angle for the rover
	#	Max steering angle found in apm_plugin_gazebo_side.cpp in the rover folder of the ardupilot/Gazebo plugin package
	max_steering_ang = 88.544 / 2 # Divide by 2 becuase They measure the ang has wheel direction left to right, we want ang from center
	if heading_diff > max_steering_ang:
		heading_diff = max_steering_ang
	if heading_diff < -1 * max_steering_ang:
		heading_diff = -1 * max_steering_ang
		
	# Find the turn ratio
	#	0 = straight	0.5 = turn right at half max strength	-1 = turn left at full max strength		etc
	turn_ratio = heading_diff / max_steering_ang
	
	# Convert the turn ratio into a yaw command to head the rover to the next waypoint
	nav_cmds['yaw'] = 1500 + ( 400 * turn_ratio)
	
	#print('Bearing: {} \t Heading: {} \t Nav Cmds: {}'.format(bearing, heading, nav_cmds))
	return nav_cmds
