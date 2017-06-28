import argparse
import subprocess
import os

parser = argparse.ArgumentParser(description='Script used for starting transport_controller.py on all of the robo VMs')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting transport_controller.py scripts on robo nodes...')

number_of_vms_on_machine = 2
number_of_machines = 1

GA_IP_ADDR = '35.9.28.201'

current_vm = 1
while current_vm <= number_of_vms_on_machine:
	cmds = """echo 'Starting transport_controller.py...';
		cd;
		source /opt/ros/indigo/setup.bash;
		source ~/simulation/ros_catkin_ws/devel/setup.bash;
		export PYTHONPATH=$PYTHONPATH:/home/simongle/simulation/ros_catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages;
		cd simulation/ros_catkin_ws/src/python_scripts/;
		echo $PYTHONPATH
		python transport_controller_v1-1.py -ip {};
		exec bash
		""".format(GA_IP_ADDR)
	cmd_str = 'xterm -title "Connection to robo1vm{}" -hold -e ssh -t -X robo1vm{}.cse.msu.edu "{}"&'.format(current_vm,current_vm,cmds)
	os.system(cmd_str)
	current_vm += 1

print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
_ = raw_input()
cmd_str = "pkill xterm"
os.system(cmd_str)
