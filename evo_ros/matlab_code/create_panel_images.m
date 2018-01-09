%% Takes the images of the best individuals from each run and creates a panel of images
%
%
% GAS 01-08-18

%%
clear all

% Configuration options
experiment_name = 'variable_sonar_placement_with_aoe_failure';
bool_save = 1;
save_file_name = strcat(experiment_name, '_winners_panel.png');



% Navigate to the analysis plots directory and read in all contents
experiment_path = strcat('./../analysis_plots/',experiment_name,'/');
run_directories = dir(experiment_path);

save_file_name = strcat(experiment_path,save_file_name);

% Iterate through the experiment runs and pull in the image of the best
% winner for each run
i = 1;
for j=1:length(run_directories)
    
    % Check to make sure that 'run' is in the name
    %   Matlab includes '.' and '..' directories by default 
    dir_name = run_directories(j).name;
    if contains(dir_name, 'run')
        
        % Grab all files 
        run_dir_path = strcat(experiment_path,dir_name,'/');
        files = dir(run_dir_path);
        
        % Find the winners image for this run
        for h=1:length(files)
            filename = files(h).name;
            
            % check to make sure the file is the diagram of the best winner
            if contains(filename, 'sensor_diagram_winner_')
                if strcmp(extractBetween(filename,length(filename)-4,length(filename)-4),'1')
                    % Append the diagram to a list of images
                    %images{i} = imread(strcat(run_dir_path,filename));
                    images{i} = fullfile(strcat(run_dir_path,filename));
                    i = i + 1;
            
                end
            end
        end
    end
end

fig = montage(images);
title(strcat(experiment_name,' Run Winners'), 'Interpreter', 'none');

if bool_save == 1
    saveas(fig,save_file_name);
end


 


 
 
 