#!/usr/bin/env python
#
# Bare bones 
# Can get or change a model state if you know the name of the model
# Can reset the world
#
# 10-24-17

import os
import rospy
import sys
import argparse

from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty

### Handle commandline arguments ###
parser = argparse.ArgumentParser(description="""Accepts a model name and new X,Y location from command line and performs a world reset then moves the model to the new location.""")
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal')
parser.add_argument('-model', type=str, help='Name of the model that is to be moved')
parser.add_argument('-X', type=int, help='X position to move the model to in the world')
parser.add_argument('-Y', type=int, help='Y position to move the model to in the world')
args= parser.parse_args()

### Default variable values ###
model_name = 'rover'
location = [5,5]

### Receive command line arguments ###
if args.model is not None:
	model_name = args.model
if args.X is not None:
	location[0] = args.X
if args.Y is not None:
	location[1] = args.Y

if args.debug:
	print("""
	Reset world and move model script:
	\t Model to move: {}
	\t New location : {}
	""".format(model_name,location))

### Wait for Gazebo Services ###
print('Waiting for gazebo services')
rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/reset_world')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')
print('All required Gazebo services are running!')

### Set up ROS service Proxies ###
getWorldProp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
getModelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
setModelState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


# Get the model state for the rover so we know what the model state looks like
model_state = getModelState(model_name, '')

# Create a modelState message
new_model_state = ModelState()
new_model_state.model_name = model_name
new_model_state.pose = model_state.pose
new_model_state.twist = model_state.twist

# Move the Rover
new_model_state.pose.position.x = location[0]
new_model_state.pose.position.y = location[1]

setModelState(new_model_state)

_=raw_input("Please enter to continue")

model_state = getModelState(model_name, '')
print('Location before reset: {}'.format(model_state.pose.position))
resetWorld()
model_state = getModelState(model_name, '')
print('Location after reset: {}'.format(model_state.pose.position))


