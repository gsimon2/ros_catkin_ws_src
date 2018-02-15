%% Grabs the N best individuals from each of the log files in the logs of interest directory
%
%
% GAS 11-14-17

%% Automation set up
clear all
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
        % Pick out best and change the ID
         [max_val, index] = max(A.Fitness);
        elite_ind = A(index,:);
        elite_ind.ID = strcat(file_name(1:end-4),'_',string(i));
        
        
        elite_indcolmissing = setdiff(elite.Properties.VariableNames, elite_ind.Properties.VariableNames);
        elitecolmissing = setdiff(elite_ind.Properties.VariableNames, elite.Properties.VariableNames);
        elite_ind = [elite_ind array2table(nan(height(elite_ind), numel(elite_indcolmissing)), 'VariableNames', elite_indcolmissing)];
        elite = [elite array2table(nan(height(elite), numel(elitecolmissing)), 'VariableNames', elitecolmissing)];
        elite = [elite; elite_ind];


        % Delete most elite
        indices = find(A.Fitness == max_val);
        A(indices,:) = [];
     end
end
% Get rid of generation column
elite.Generation = [];

% Get rid of raw fitness column
elite.RawFitness = [];

% Get rid of any columns that have 'Var' in it. Comes from how Matlab
% imports RawFitness
for k=length(elite.Properties.VariableNames):-1:1
   if contains(elite.Properties.VariableNames(k), 'Var')
       elite.(k) = [];
   end
end

writetable(elite, 'best_individuals.txt');
head(elite)
cd('../')
 


 
 
 