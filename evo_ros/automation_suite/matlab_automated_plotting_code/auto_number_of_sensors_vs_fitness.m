%% Compares the number of sensors vs the fitness
%
%
% GAS 11-28-17

%% Automation set up
fig = figure(1);
save_dir = strcat('~/simulation/ros_catkin_ws/src/evo_ros', plot_dir);
title_string = file_name(1:end-4);
title_string = strcat(title_string,' Generation vs Fitness');
title(title_string, 'Interpreter', 'none');
save_file_name = strcat(file_name(1:end-4),'_num_sensors_vs_fitness.png')

%% Read in table and set up plotting arrays
cd('~/simulation/ros_catkin_ws/src/evo_ros/GA/logs');
table = readtable(file_name);
cd(save_dir)
%cd('~/simulation/ros_catkin_ws/src/evo_ros/automation_suite')
hold on
avg = [];
best = [];
generation = [];

%% Set up sensor tracking
for i=1:10
    sensor(i) = {0};
end



 %% Dynamically figure out population size and generation count
 A = table(table.Generation == 0, :);
 population_size = height(A);
 gen_count = height(table) / population_size;
 
%% loop through each generation creating a scattor plot
for i=0:gen_count-1
    
    % Create a table of just the individuals from this generation
    A = table(table.Generation == i, :);
    
    for j=1:height(A)
        number_of_sensors = A.NumberOfSonar(j);
        sensor(number_of_sensors) = {[sensor{number_of_sensors}, A.Fitness(j)]};
    end
    
    
    % For each generation add to the arrays tracking the average and best
    % fitnesss as well as the one for generations
    avg = [avg, mean(A.Fitness)];
    best = [best, max(A.Fitness)];
    generation = [generation, A.Generation(1)];
    
    % Create a scattor plot for this generatin
    %   for the two arrays need to be the same size since it does pairwise
    %   matching
    scatter(A.Generation,A.Fitness);
end

%% Plot average line in red and best in blue
plot(generation,avg,'r')
plot(generation,best,'b')
legend('Average Fitness','Best Fitness', 'location', 'best')
xlabel('Generation')
ylabel('Fitness')


fig2 = figure(2);

average_per_sensor = zeros(1,10);
best_per_sensor = zeros(1,10);
bar_data = [];
%% Clean up inial values for sensor array
for i=1:10
    sensor{i}(1) = [];
    if ~(length(sensor{i}) == 0)
        average_per_sensor(i) = mean(sensor{i});
        best_per_sensor(i) = max(sensor{i});
        bar_data = [bar_data; [average_per_sensor(i) best_per_sensor(i)]];
    else
        average_per_sensor(i) = 0;
        best_per_sensor(i) = 0;
        bar_data = [bar_data; [0 0]];
    end
end

bar(bar_data)
title_string = strcat(file_name(1:end-4),' Number of Sensors vs Fitness');
title(title_string, 'Interpreter', 'none');
xlabel('Number of Sensors')
ylabel('Fitness')
legend('Average Fitness', 'Best Fitness', 'location', 'best')

saveas(fig2,save_file_name);
close(fig)
close(fig2)