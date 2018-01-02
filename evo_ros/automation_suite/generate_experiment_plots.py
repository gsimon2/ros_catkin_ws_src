# Generate Plots
# 	Runs all of the matlab plotting scripts in the matlab_automated_plotting_code directory for the set experiment and run number
#	Then stores all of the plots in the analysis_plots directory sorted by experiment and run
#
# GAS 11-8-17
import argparse
import subprocess
import os
from os import listdir
from os.path import isfile, join

# Set up parser
parser = argparse.ArgumentParser(description='Script used for to generate plots for all runs of an experiment')
parser.add_argument('-e', '--experiment', type=str, help='Name of the experiment')
parser.add_argument('-r', '--runs', type=int, help='Number of runs')
args= parser.parse_args()

# Set up variables that determine the file name of the data log
if args.experiment is None:
	experiment_name = 'variable_sonar_placement_with_failure'
else:
	experiment_name = args.experiment
if args.runs is None:
	number_of_runs = 10
else:
	number_of_runs = args.runs

# Loop through each running each of the plotting scripts in the matlab_autmated_code dir
for run_number in range(1,number_of_runs+1):
	# Build data log file name from variables
	data_log_name = experiment_name + '_run' + str(run_number) + '.dat'
	print('Plotting for data file: {}'.format(data_log_name))
	
	# Load in the names of all of the plotting scripts available
	path_to_plotting_scripts = './matlab_automated_plotting_code/'
	plotting_scripts = [f for f in listdir(path_to_plotting_scripts) if isfile(join(path_to_plotting_scripts, f))]
	
	# Make sure that a directory for this experiment has been made
	experiment_directory = './../analysis_plots/' + experiment_name
	if not os.path.isdir(experiment_directory):
		print('No directory for experiment...Creating now.')
		os.makedirs(experiment_directory)
	
	# Make sure that a directory for this run has been made
	run_directory = experiment_directory + '/run' + str(run_number)
	if not os.path.isdir(run_directory):
		print('No directory for run...Creating now.')
		os.makedirs(run_directory)
		
	plot_dir = run_directory[4:] # path to run_directory from the evo_ros directory (how matlab code wants it)
	
	# Run all of the plotting scripts
	for current_plotting_script in plotting_scripts:
		print(current_plotting_script)
		cmd_str = """/usr/local/MATLAB/R2017b/bin/matlab -nodisplay -nosplash -nodesktop -r \"file_name='{}';plot_dir='{}';run('./matlab_automated_plotting_code/{}');exit;\"""".format(data_log_name,plot_dir,current_plotting_script)
		os.system(cmd_str)
os.system('reset')
print('\n\n Done with plotting!\n\n')


