#!/usr/bin/env python

### Code comes from:
# 	http://python.dronekit.io/examples/mission_basic.html
###
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import std_msgs.msg
import rospy

from gazebo_msgs.srv import GetWorldProperties
from std_srvs.srv import Empty


connection_string = '127.0.0.1:14550'
flying_height = 4
vehicle = ''
# Boolean representing if an ending condition has been triggered or not
simulation_end = False
max_sim_time = 240


def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
	specified `original_location`. The returned Location has the same `alt` value
	as `original_location`.
	
	The function is useful when you want to move the vehicle around specifying locations relative to 
	the current vehicle position.
	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
	For more information see:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	"""
	earth_radius=6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
	
	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.
	
	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
	"""
	Gets distance in metres to the current waypoint. 
	It returns None for the first waypoint (Home location).
	"""
	
	global vehicle
	nextwaypoint = vehicle.commands.next
	if nextwaypoint==0:
		return None
	missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z
	targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
	return distancetopoint


def download_mission():
	"""
	Download the current mission from the vehicle.
	"""
	
	global vehicle
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready() # wait until download is complete.



def adds_square_mission(aLocation, aSize):
	"""
	Adds a takeoff command and four waypoint commands to the current mission. 
	The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
	
	The function assumes vehicle.commands matches the vehicle mission state 
	(you must have called download at least once in the session and after clearing the mission)
	"""	
	
	cmds = vehicle.commands
	
	print " Clear any existing commands"
	cmds.clear() 
	
	print " Define/add new commands."
	# Add new commands. The meaning/order of the parameters is documented in the Command class. 
	 
	#Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
	
	#Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
	point1 = get_location_metres(aLocation, aSize, -aSize)
	point2 = get_location_metres(aLocation, aSize, aSize)
	point3 = get_location_metres(aLocation, -aSize, aSize)
	point4 = get_location_metres(aLocation, -aSize, -aSize)
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 5))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 5))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 5))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 5))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 5))    
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 5))  
	print " Upload new commands to vehicle"
	cmds.upload()


def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""
	
	global vehicle
	print "Basic pre-arm checks"
	# Don't let the user try to arm until autopilot is ready
	while not vehicle.is_armable:
		print " Waiting for vehicle to initialise..."
		time.sleep(1)
	
		
	print "Arming motors"
	# Copter should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	
	while not vehicle.armed:      
		print " Waiting for arming..."
		time.sleep(1)
	
	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
	
	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
	#  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print " Altitude: ", vehicle.location.global_relative_frame.alt      
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
			print "Reached target altitude"
			break
		time.sleep(1)


def is_vehicle_home():
	global vehicle
	
	bool_home = True
	if round(vehicle.home_location.lat,4) != round(vehicle.location.global_frame.lat,4):
		bool_home = False
	else:
		print('lat locked')
	if round(vehicle.home_location.lon,4) != round(vehicle.location.global_frame.lon,4):
		bool_home = False
	else:
		print('lon locked')
	if round(vehicle.home_location.alt,3) != round(vehicle.location.global_frame.alt/1000,3):
		bool_home = False
	else:
		print('alt locked')
	return bool_home
	
def software_ready_callback(data):
	global vehicle
	print("Software is ready! Starting sim....")
	
	time.sleep(5)
	# Connect to the Vehicle
	print 'Connecting to vehicle on: %s' % connection_string
	vehicle = connect(connection_string, wait_ready=True)

	global simulation_end
	simulation_end = False
	time_fitness = 0
	
	# Get the beginning time for the simulation
	# Error handling for if a required process crashes
	#	Mainly for Gazebo, which has a tendency to crash during
	#	long runs
	try:
		begin_time = getWorldProp().sim_time 
	except:
		print('Required processes has failed. Sending reset message to software manager')
		sim_result_pub.publish(-2)
	
	# Send a ready message on the evaluation start topic to let controller
	#	know that the simulation enviroment is ready
	rospy.set_param('simulation_running', True)


	# Wait until a simulation end event is triggered
	#	or for the max simulation time to be reached
	sim_timeout = False
	print 'Create a new mission (for current location)'
	adds_square_mission(vehicle.location.global_frame,10)
	
	
	# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
	arm_and_takeoff(flying_height)
	
	print "Starting mission"
	# Reset mission set to first (0) waypoint
	vehicle.commands.next=0
	
	# Set mode to AUTO to start mission
	vehicle.mode = VehicleMode("AUTO")
	
	
	# Monitor mission. 
	# Demonstrates getting and setting the command number 
	# Uses distance_to_current_waypoint(), a convenience function for finding the 
	#   distance to the next waypoint.
	
		
	while not is_vehicle_home():
		nextwaypoint=vehicle.commands.next
		if nextwaypoint==6:
			print 'Returning to launch'
			vehicle.mode = VehicleMode("RTL")	
		else:
			print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
		time.sleep(1)
	
	print('mission complete!')
	current_time = getWorldProp().sim_time 
	
	vehicle.mode = VehicleMode("STABILIZE")
	time.sleep(4)
	vehicle.armed = False
	
	#Close vehicle object before exiting script
	print "Close vehicle object"
	vehicle.close()
	
	rospy.set_param('simulation_running', False)

	# If the vehicle was able to finish successfully, give it a time bonus
	#	Else send back a result of -1 indicating a collision
	#	Or -2 indicating that the simulation took too long to finish sucessfully
	global percent_complete
	percent_complete = 100
	if percent_complete == 100:
		total_sim_time = current_time - begin_time
		time_fitness = (max_sim_time - total_sim_time) / max_sim_time
	else:
		if sim_timeout == True:
			time_fitness = -2
		else:
			time_fitness = -1
		

	
	# Reset simulation
	print("Attempting to reset...")
	resetWorld()
	time.sleep(3)
	print("Reset!")
	
	# Send simulation results
	rospy.set_param('percent_complete', percent_complete)
	sim_result_pub.publish(time_fitness)
	
	
### Wait for Gazebo Services ###
print('Waiting for gazebo services')
rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/reset_world')
rospy.wait_for_service('/gazebo/reset_simulation')
rospy.wait_for_service('/gazebo/pause_physics')
rospy.wait_for_service('/gazebo/unpause_physics')


### Set up ROS service Proxies ###
getWorldProp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

### Set up ROS subscribers and publishers ###
rospy.init_node('copter_controller',anonymous=False)
sim_start_sub = rospy.Subscriber('software_ready', std_msgs.msg.Empty, software_ready_callback)
sim_result_pub = rospy.Publisher('evaluation_result', std_msgs.msg.Float64, queue_size=1)

rospy.spin()
