import argparse
import subprocess
import os
import yaml
import re

parser = argparse.ArgumentParser(description='Script used for starting transport_controller.py on all of the robo VMs')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting software_manager.py scripts on Group 2 robo nodes...')

name_of_script = "software_manager.py"
GA_IP_ADDR = '35.9.28.231'
script_arguments = "-ip {} -mw".format(GA_IP_ADDR)
ga_hostname = 'robo4vm1'

work_nodes_file_name = 'group_2_nodes.yml'

with open(os.path.dirname(os.path.abspath(__file__)) + '/{}'.format(work_nodes_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

for worker in cfg['worker_list']:
	print(str(worker))
	
	if str(worker) is ga_hostname:
		continue

	#Start Software Manager
	cmds = """echo {}...;
		cd;
		source /opt/ros/indigo/setup.bash;
		source ~/simulation/ros_catkin_ws/devel/setup.bash;
		export PYTHONPATH=$PYTHONPATH:/home/simongle/simulation/ros_catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages;
		rosrun evo_ros {} {};
		exec bash
		""".format(name_of_script, name_of_script, script_arguments)
	cmd_str = """xterm -hold -title "Connection to {}" -e 'ssh -t -X {}.cse.msu.edu "{}"'&""".format(worker,worker,cmds)
	os.system(cmd_str)


print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
_ = raw_input()
cmd_str = "pkill xterm"
os.system(cmd_str)
