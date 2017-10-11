from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import sys
import rospy
import tf
import argparse
from pymavlink import mavutil
from dronekit_functions import get_location_metres, get_distance_metres, distance_to_current_waypoint, download_mission, adds_square_mission, is_vehicle_home, angle_to_current_waypoint, condition_yaw
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from mavros_msgs.msg import OverrideRCIn
import std_msgs.msg
import numpy as np
import message_filters
from obstacle_avoidance_functions import partition_vision, check_vision, parse_genome, findMiddle, ErleRover_Obstacle_Avoidance, sonar_avoidance
import mav_msgs.msg as mav_msgs
from gazebo_msgs.msg import LinkStates
from rover_ga.msg import waypoint

from threading import Thread,Lock


class globalVars():
    pass

G = globalVars() #empty object to pass around global state
G.lock = Lock() #not really necessary in this case, but useful none the less
G.value = 0
G.kill = False

def display_Current_WP_Bearing():
	while True:
		if G.kill:
			G.kill = False
			return
		ang = angle_to_current_waypoint(vehicle)
		print('Bearing to next waypoint: {}'.format(ang))
		time.sleep(1)


connection_string = '127.0.0.1:14551'

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)


# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print " Waiting for home location ..."
        vehicle.home_location = vehicle.location.global_frame
# We have a home location, so print it!        
print "\n Home location: %s" % vehicle.home_location


print 'Create a new mission (for current location)'
adds_square_mission(vehicle, vehicle.location.global_frame,5)
time.sleep(1)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=0
time.sleep(1)

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("GUIDED")


t = Thread(target=display_Current_WP_Bearing, args=())
t.start()


while True:
	command = raw_input('Enter bearing')
	if command == 'q':
		G.kill = True
		break
	else:
		condition_yaw(vehicle, float(command))
		time.sleep(1)
	
	
print('Done!')


