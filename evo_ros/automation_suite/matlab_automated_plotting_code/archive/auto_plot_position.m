%% Plot positions of sensors on rover
%
%
% GAS 11-8-17

%% Automation set up
fig = figure;
save_dir = strcat('~/simulation/ros_catkin_ws/src/evo_ros', plot_dir);
title_string = file_name(1:end-4);
title_string = strcat(title_string,' Sensor Position');
title(title_string, 'Interpreter', 'none');
save_file_name = strcat(file_name(1:end-4),'_sensor_position.png')

%% Read in table and set up plotting arrays
cd('~/simulation/ros_catkin_ws/src/evo_ros/GA/logs');
table = readtable(file_name);
cd(save_dir)
x1 = [];
y1 = [];
x2 = [];
y2 = [];
fitness = [];

 %% Dynamically figure out population size and generation count
 A = table(table.Generation == 0, :);
 population_size = height(A);
 gen_count = height(table) / population_size;
 hold on
%% loop through each generation tracking the positions for each sensor 
% and plotting a line between the two points
for i=0:gen_count-1
    
     % Create a table of just the individuals from this generation
    A = table(table.Generation == i, :);
    
    for j=1:population_size
        
        % If one sensor just append the ang and fitness to the arrays
        if A.NumberOfSonar(1) == 1
            x1 = [x1, A.S1_P_X(j)];
            y1 = [y1, A.S1_P_Y(j)];
            fitness = [fitness, A.Fitness(j)];
        
        % If two sensors append the ang of each sensor to its own array
        %   and plot a line connecting the two sensors
        else
            x1 = [x1, A.S1_P_X(j)];
            y1 = [y1, A.S1_P_Y(j)];
            x2 = [x2, A.S2_P_X(j)];
            y2 = [y2, A.S2_P_Y(j)];
            fitness = [fitness, A.Fitness(j)];
            %plot([A.S1_P_Y(j) A.S2_P_Y(j)], [A.S1_P_X(j) A.S2_P_X(j)], 'k')
            %plot([A.S1_P_X(j) A.S2_P_X(j)], [A.S1_P_Y(j) A.S2_P_Y(j)], 'k')
        end
        
    end

end

%X = x1;
%Y = y1;
%scatter(X,Y)

s(1) = scatter(y1,x1, 'b');
s(2) = scatter(y2,x2, 'r');
legend(s([1,2]),{'Sensor 1', 'Sensor 2'}, 'location', 'best')
xlabel('Left / Right from center (meters)')
ylabel('Forward / Back from center (meters)')
hold off

saveas(fig,save_file_name);
close(fig)