%% Studying the fitness effect of the angle of the sonar
%
%
% GAS 11-8-17

%% Automation set up
fig = figure;
save_dir = strcat('~/simulation/ros_catkin_ws/src/evo_ros', plot_dir);
title_string = file_name(1:end-4);
title_string = strcat(title_string,' Fitness vs Angle');
title(title_string, 'Interpreter', 'none');
save_file_name = strcat(file_name(1:end-4),'_angle_vs_fitness.png')

%% Read in table and set up plotting arrays
cd('~/simulation/ros_catkin_ws/src/evo_ros/GA/logs');
table = readtable(file_name);
cd(save_dir)
ang1 = [];
ang2 = [];
fitness = [];
avg = [];
best = [];
generation = [];


 %% Dynamically figure out population size and generation count
 A = table(table.Generation == 0, :);
 population_size = height(A);
 gen_count = height(table) / population_size;
 hold on
%% loop through each generation tracking the ang for each sensor 
% and plotting a line between the two points
for i=0:gen_count-1
    
     % Create a table of just the individuals from this generation
    A = table(table.Generation == i, :);
    
    for j=1:population_size
        
        % If one sensor just append the ang and fitness to the arrays
        if A.NumberOfSonar(1) == 1
            ang1 = [ang1, A.S1_O(j)];
            fitness = [fitness, A.Fitness(j)];
        
        % If two sensors append the ang of each sensor to its own array
        %   and plot a line connecting the two sensors
        else
            ang1 = [ang1, A.S1_O(j)];
            fitness = [fitness, A.Fitness(j)];
            ang2 = [ang2, A.S2_O(j)];
            plot([A.S1_O(j) A.S2_O(j)], [A.Fitness(j) A.Fitness(j)], 'k')
        end
        
    end

end

s(1) = scatter(ang1,fitness, 'b');
s(2) = scatter(ang2,fitness, 'r');
legend(s([1,2]),{'Sensor 1', 'Sensor 2'}, 'location', 'best')
xlabel('Angle')
ylabel('Fitness')
hold off

saveas(fig,save_file_name);
close(fig)

