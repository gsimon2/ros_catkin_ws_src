import argparse
import subprocess
import os

parser = argparse.ArgumentParser(description='Script used for making the ROS ws on all of the robo VMs')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Making ROS WS on robo nodes...')

number_of_vms_on_machine = 8
number_of_machines = 1
name_of_script = "software_manager.py"
GA_IP_ADDR = '35.9.28.201'
#script_arguments = "-ip {}".format(GA_IP_ADDR)

current_vm = 2
while current_vm <= number_of_vms_on_machine:
	cmds = """echo Making ROS WS...;
		cd;
		source /opt/ros/indigo/setup.bash;
		source ~/simulation/ros_catkin_ws/devel/setup.bash;
		export PYTHONPATH=$PYTHONPATH:/home/simongle/simulation/ros_catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages;
		cd simulation/ros_catkin_ws/
		catkin_make
		exec bash
		"""
	cmd_str = """xterm -hold -title "Connection to robo1vm{}" -e 'ssh -t -X robo1vm{}.cse.msu.edu "{}"'&""".format(current_vm,current_vm,cmds)
	os.system(cmd_str)
	current_vm += 1

print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
_ = raw_input()
cmd_str = "pkill xterm"
os.system(cmd_str)
