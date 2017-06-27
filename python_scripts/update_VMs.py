import argparse
import subprocess
import os

parser = argparse.ArgumentParser(description='Script used for updating the ros_catkin_ws code on all of the robo VMs from the github repo')
#parser.add_argument('-r', '--remote', action='store_true', help='Use when not on MSU Engineering network. SSH\'s into arctic server before going to robo servers')
parser.add_argument('-d', '--debug', action='store_true', help='Print extra output to terminal, spawn subprocesses in xterm for seperated process outputs')
parser.add_argument('-p', '--password', type=str, help='Users password on remote machine. This is a required parameter')
args= parser.parse_args()

print('Starting update scripts on robo nodes...')

current_vm = 1
cmd_str = """xterm -title 'Update Script" -hold -e '
			

		"""



		if args.debug:
			cmd_str = """xterm -title 'MAVProxy' -hold  -e '
				source ~/simulation/ros_catkin_ws/devel/setup.bash;
				cd ~/simulation/ardupilot/APMrover2;
				echo \"param load ~/simulation/ardupilot/Tools/Frame_params/3DR_Rover.param\";
				echo
				echo \" (For manual control) - param set SYSID_MYGCS 255\";
				echo
				echo \" (For script control) - param set SYSID_MYGCS 1\";
				../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo'&"""
			os.system(cmd_str)
