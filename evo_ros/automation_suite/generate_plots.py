import argparse
import subprocess
import os
from os import listdir
from os.path import isfile, join

# Set up variables that determine the file name of the data log
experiment_name = 'double_sonar_with_complete_failure'
run_number = '2'

# Build data log file name from variables
data_log_name = experiment_name + '_run' + run_number + '.dat'
print('Plotting for data file: {}'.format(data_log_name))

# Load in the names of all of the plotting scripts available
path_to_plotting_scripts = './matlab_automated_code/'
plotting_scripts = [f for f in listdir(path_to_plotting_scripts) if isfile(join(path_to_plotting_scripts, f))]

# Make sure that a directory for this experiment has been made
experiment_directory = './../analysis_plots/' + experiment_name
if not os.path.isdir(experiment_directory):
	print('No directory for experiment...Creating now.')
	os.makedirs(experiment_directory)

# Make sure that a directory for this run has been made
run_directory = experiment_directory + '/run' + run_number
if not os.path.isdir(run_directory):
	print('No directory for run...Creating now.')
	os.makedirs(run_directory)
plot_dir = run_directory[4:] # path to run_directory from the evo_ros directory (how matlab code wants it)


for current_plotting_script in plotting_scripts:
	print(current_plotting_script)
	cmd_str = """/usr/local/MATLAB/R2017b/bin/matlab -nodisplay -nosplash -nodesktop -r \"file_name='{}';plot_dir='{}';run('./matlab_automated_code/{}');exit;\"""".format(data_log_name,plot_dir,current_plotting_script)
	os.system(cmd_str)



