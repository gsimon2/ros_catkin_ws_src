import argparse
import subprocess
import os
import yaml
import re

parser = argparse.ArgumentParser(description='Script used for updating the ros_catkin_ws code on all of the robo VMs from the github repo')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
#parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
#parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting cleaning scripts on robo nodes...')


work_nodes_file_name = 'active_nodes.yml'

with open(os.path.dirname(os.path.abspath(__file__)) + '/{}'.format(work_nodes_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

for worker in cfg['worker_list']:
	print(str(worker))
	cmds = """echo 'Cleaning gazebo logs from vms';
		rm -rf ~/.gazebo/log/*;
		rm ~/simulation/ardupilot/APMrover2/logs/*;
		rosclean purge;
		df;
		exec bash
		"""
	cmd_str = 'xterm -title "Connection to {}" -hold -e ssh -t -X {}.cse.msu.edu "{}"&'.format(worker,worker,cmds)
	os.system(cmd_str)

print('Script finished! \n')

print('Press enter to close all xterm windows and close this script...')
_ = raw_input()
cmd_str = "pkill xterm"
os.system(cmd_str)

