#!/usr/bin/env python
import os
import fileinput
import subprocess



import socket
import time




# Copies the base_erlecopter.xacro file in the same directory with name of the host computer name + '_erlecopter.xacro'
# 	param - str_host_name - string of the name of the host computer
#	returns - str_copter_file - string of the complete path for the new file
def copy_base_copter_file(str_host_name):
	str_copter_file_name = str_host_name + '_erlecopter.xacro' #New file name = This computers name + '_erlecopter.xacro'
	str_user_home_dir = 'locate ~ --limit 1'
	str_user_home_dir = subprocess.check_output(str_user_home_dir,stderr=subprocess.STDOUT,shell=True) #gets the home directory of the current user
		#needs this if multiple users on the same machine have the ros_catkin directory
	
	str_user_home_dir = str_user_home_dir.rstrip() #Remove newline character from returned path
	
	###
	print('Home dir: {}'.format(str_user_home_dir))
	
	str_copter_file_path = "$(dirname $(locate -ir '{}/.*base_erlecopter.xacro'))".format(str_user_home_dir) #Gets the dir name (path) that the base_erlecopter.xacro file is in
		#locate -ir  = use regex expression to find a file that starts with the users home dir and ends with base_erlecopter.xacro
	
	str_copter_file = str_copter_file_path + '/' + str_copter_file_name
	
	###
	print('Copter path: {} \n Copter file: {}'.format(str_copter_file_path, str_copter_file))
	
	cmd_str = 'cp $(find ~ -name base_erlecopter.xacro) ' +  str_copter_file #Copy the file
	os.system(cmd_str)
	
	str_copter_file = subprocess.check_output('find ~ -name {} | head -n 1'.format(str_copter_file_name),stderr=subprocess.STDOUT,shell=True) #gets the abs path from OS
	str_copter_file = str_copter_file.rstrip() #Remove newline character from returned path
	
	print("Copter model file: {}".format(str_copter_file))
	return str_copter_file

# Opens copter.xacro file, splits it where the add sensors tag is, and adds the code for a lidar sensor
#	param - str_copter_file - string of the copter.xacro file path
#	param - pos - position of sensor in respect to copter
#	param - orient  orientation of sensor in respect to copter
def add_lidar_copter(str_copter_file, pos = [0.13, 0.0, 0.02], orient = [0,0,0]):
	#print('Modifying file: {}!'.format(str_rover_file))
	
	with open(str_copter_file) as f:
		content = f.read()
		
	x = content.split('<!-- ADD SENSORS HERE -->')
	
	lidar_sensor = """<!-- ADD SENSORS HERE -->
	
  <xacro:include filename="$(find ardupilot_sitl_gazebo_plugin)/urdf/sensors/lidar_sensor.urdf.xacro" />
   <xacro:lidar_sensor
    name="sonar2"
    parent="base_link"
    ros_topic="sonar_front"
    update_rate="10"
    min_range="0.01"
    max_range="10.0"
    field_of_view_horizontal="${{70*M_PI/180}}"
    field_of_view_vertical="${{1*M_PI/180}}"
    ray_count_horizontal="140"
    ray_count_vertical="1"
    sensor_mesh="lidar_lite_v2_withRay/meshes/lidar_lite_v2_withRay.dae">
    <origin xyz="{} {} {}" rpy="{} {} {}"/>
  </xacro:lidar_sensor>
""".format(pos[0], pos[1], pos[2], orient[0], orient[1], orient[2])

	content = x[0] + lidar_sensor + x[1]
	
	with open(str_copter_file, "w") as f:
		f.write(content)
	
str_host_name = socket.gethostname()
print('Host name: {}'.format(str_host_name))

print('Copying base_copter file')

str_file = copy_base_copter_file(str_host_name)
time.sleep(1)
add_lidar_copter(str_file, [0.13, 0.0, 0.02], [0, 0, 0])

print('File copy complete')
