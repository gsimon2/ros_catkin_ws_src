%% Plots the images of the best individuals from each log file and overlays the images on top
%   of one another to display the common viewing angles shared by the
%   individuals
%
% GAS 01-09-18

%%
clear all

% Configuration options
experiment_name = '6_sonar_symmetric_placement_with_complete_failure';
bool_save = 1;
save_file_name = strcat(experiment_name, '_sensor_viewing_area_overlay.png');
num_of_ind = 1; % Num of individuals to include from each log file

% Navigate to the logs directory and read in all contents
log_file_dir_path = '../GA/logs/';
log_files = dir(log_file_dir_path);

% Prepare the save file name
experiment_path = strcat('./../analysis_plots/',experiment_name,'/');
save_file_name = strcat(experiment_path,save_file_name);

% Iterate through the logs dir and pull in the logs for this
% experiment
i = 1;
for j=1:length(log_files)
    
    % Find the logs for this exp
    log_file_name = log_files(j).name;
    if contains(log_file_name, experiment_name)
        experiment_logs{i} = fullfile(strcat(log_file_dir_path,log_file_name));
        i = i + 1;
    end
end

%% Iterate through the experiment logs and pull out the best N (config option)
%   number of individuals from each run log
winning_individuals = table;
for j=1:length(experiment_logs)
   these_winning_individuals = find_best_individuals_from_log(experiment_logs{j},num_of_ind,experiment_name,j);
   winning_individuals = [winning_individuals; these_winning_individuals];
end



%% Create a figure of the rover diagram
diagram = figure;
plot_rover_diagram();

%% Label experiment name
text(0,-20,experiment_name, 'Interpreter', 'none',...
        'HorizontalAlignment', 'center')


%% Overlay sensors from each individual
for i=1:height(winning_individuals)
%for i=2:2
    ind = winning_individuals(i,:);
    
    % Iterate through the sensors on this ind
    for j=4:3:width(ind)-2
        color = 'R';
        ind_angle = ind{1, j} * -1; % -1 because gazebo flips orienation
        ind_pos_x = ind{1,j+1} * 100; % times 100 to conver meters (gazebo) to cm (shown on plot)
        ind_pos_y = ind{1,j+2} * -100; % negative again to deal with Gazebos flip
        overlay_sensor(ind_pos_y, ind_pos_x,ind_angle, color); % swapping x and y values because Gazebo flips these axis

    end
end

if bool_save == 1
    saveas(diagram,save_file_name);
end





function best_inds = find_best_individuals_from_log(log_file_path, num_of_ind, experiment_name,j)
    elite = table; 
    
    %% Read in table
    log_data = readtable(log_file_path);

     %% Dynamically figure out population size and generation count
     A = log_data(log_data.Generation == 0, :);
     population_size = height(A);
     gen_count = round(height(log_data) / population_size);

     % Only interested in last gen
     A = log_data(log_data.Generation == gen_count-1, :);

     %% Look at individuals in the last gen and find best
     for i=1:num_of_ind
        % Pick out best and change the ID
         [max_val, index] = max(A.Fitness);
        elite_ind = A(index,:);
        elite_ind.ID = strcat(experiment_name,'_',string(j));
        
        
        elite_indcolmissing = setdiff(elite.Properties.VariableNames, elite_ind.Properties.VariableNames);
        elitecolmissing = setdiff(elite_ind.Properties.VariableNames, elite.Properties.VariableNames);
        elite_ind = [elite_ind array2table(nan(height(elite_ind), numel(elite_indcolmissing)), 'VariableNames', elite_indcolmissing)];
        elite = [elite array2table(nan(height(elite), numel(elitecolmissing)), 'VariableNames', elitecolmissing)];
        elite = [elite; elite_ind];


        % Delete most elite
        indices = find(A.Fitness == max_val);
        A(indices,:) = [];
     end
     
     %% Clean up the winning_individuals table
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

    best_inds = elite;
end

function plot_rover_diagram()
    hold on;
    
    pw = 5; % placement width - width of outside ledge to place sensor on 
    % Outside border
    rectangle('Position',[-15 -25 30 50]); % start x,y , width, height
    
    % Mid line
    plot([-15 15],[0 0], 'color', 'black')
    
    % Region 1
    rectangle('Position',[-10 20 20 5])
    text(-5, 22.5, 'Region 1', 'color', 'black')
    
    % Region 2
    rectangle('Position',[-15 0 5 20])
    text(-14, 12, 'R2', 'color', 'black')
    
    % Region 3
    rectangle('Position',[10 0 5 20])
    text(11, 12, 'R3', 'color', 'black')
    
    % Region 4
    %rectangle('Position',[-10 20 20 5])
    text(-14, 22.5, 'R4', 'color', 'black')
    
    % Region 5
    %rectangle('Position',[-10 20 20 5])
    text(11, 22.5, 'R5', 'color', 'black')
    
    % Fill in resricted areas
    rectangle('Position',[-10 0 20 20],...
        'FaceColor', [0.85 0.85 0.85])
    rectangle('Position',[-15 -25 30 25],...
        'FaceColor', [0.85 0.85 0.85])
    
    % Label front of rover
    text(0,27,'Front', 'Interpreter', 'none',...
        'HorizontalAlignment', 'center')
    
    % Label back of rover
    text(0,-27,'Back', 'Interpreter', 'none',...
        'HorizontalAlignment', 'center')
    
    
    axis([-30 30 -30 40])
    axis equal
    
    
end

 function b = overlay_sensor(pos_x, pos_y, angle, color)
    hold on;
    
    
    sensor_color = color;
    vision_cone_color = color;
    alpha_value = 0.3;
    viewing_ang = 15;
    mid_line_length = 7.5;
    cone_lines_length = 20;
    
    % Marker for the sensor
   % plot1= plot(pos_x, pos_y,...
    %    'r.',...
     %   'color', sensor_color,...
      %  'MarkerSize', 30);
    
    sensor_loc = scatter(pos_x, pos_y, 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
    sensor_loc.MarkerFaceAlpha = alpha_value;
    sensor_loc.MarkerEdgeAlpha = alpha_value;
     
    % Line indicating the direction that the sensor is facing
    plot1 = plot([pos_x (mid_line_length * sind(angle)+pos_x)],...
         [pos_y (mid_line_length * cosd(angle)+pos_y)],...
         'color', sensor_color);
    plot1.Color(4) = alpha_value;
   
   
     
    % Left line of the vision cone
    plot1 = plot([pos_x (cone_lines_length * sind(angle - viewing_ang)+pos_x)],...
         [pos_y (cone_lines_length * cosd(angle - viewing_ang)+pos_y)],...
         'color', vision_cone_color);
    plot1.Color(4) = alpha_value;
    
     
     % Right line of the vision cone
    plot1 = plot([pos_x (cone_lines_length * sind(angle + viewing_ang)+pos_x)],...
         [pos_y (cone_lines_length * cosd(angle + viewing_ang)+pos_y)],...
         'color', vision_cone_color);
    plot1.Color(4) = alpha_value;
    
    
    
    % Fill a the viewing angle cone
    leftx = [pos_x (cone_lines_length * sind(angle - viewing_ang)+pos_x)];
    lefty = [pos_y (cone_lines_length * cosd(angle - viewing_ang)+pos_y)];
    rightx = [pos_x (cone_lines_length * sind(angle + viewing_ang)+pos_x)];
    righty = [pos_y (cone_lines_length * cosd(angle + viewing_ang)+pos_y)];
    
    x2 = [leftx, fliplr(rightx)];
    inBetween = [lefty, fliplr(righty)];
    filled_area = fill(x2, inBetween, color, 'LineStyle', 'none');
    alpha(filled_area, alpha_value);
    
    
 
    
    

end


 
 
 