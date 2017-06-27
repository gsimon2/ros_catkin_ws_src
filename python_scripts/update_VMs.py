import argparse
import subprocess
import os

parser = argparse.ArgumentParser(description='Script used for updating the ros_catkin_ws code on all of the robo VMs from the github repo')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting update scripts on robo nodes...')

current_vm = 2
cmd_str = """xterm -title 'Update Script" -hold -e '
			sshpass -p '{}' ssh -X robo1vm{}.cse.msu.edu;
			echo \"Login Successful!\"
		'&""".format(args.password, current_vm)

mavproxy = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)
#os.system(cmd_str)

print('Script finished!')
