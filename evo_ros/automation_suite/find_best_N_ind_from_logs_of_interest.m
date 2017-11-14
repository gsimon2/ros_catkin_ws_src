%% Grabs the N best individuals from each of the log files in the logs of interest directory
%
%
% GAS 11-14-17

%% Automation set up
clear all;
num_of_ind = 3;
elite = table; 
%save_file_name = strcat(file_name(1:end-4),'_generation_vs_fitness.png');

cd('./logs_of_interest');
files = dir('*.dat');

for j=1:length(files)
    file_name = files(j).name;

    %% Read in table
    %cd('~/simulation/ros_catkin_ws/src/evo_ros/GA/logs');
    log_data = readtable(file_name);

     %% Dynamically figure out population size and generation count
     A = log_data(log_data.Generation == 0, :);
     population_size = height(A);
     gen_count = height(log_data) / population_size;

     % Only interested in last gen
     A = log_data(log_data.Generation == gen_count-1, :);


     %% Look at individuals in the last gen and find best
     for i=1:num_of_ind
        [max_val, index] = max(A.Fitness);
        elite_ind = A(index,:);
        elite_ind.ID = strcat(file_name(1:end-4),'_',string(i));
        elite = [elite; elite_ind];


        % Delete most elite
        indices = find(A.Fitness == max_val);
        A(indices,:) = [];
     end
end
 cd('../')
 
 % Get rid of generation column
 elite = elite(:,2:end);

 
 
 