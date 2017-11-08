#!/bin/bash
echo Generating Plots...;
/usr/local/MATLAB/R2017b/bin/matlab -nodisplay -nosplash -nodesktop -r "file_name='double_sonar_with_knockout_run2.dat';run('./matlab_automated_code/auto_angle_vs_fitness.m');exit;"

