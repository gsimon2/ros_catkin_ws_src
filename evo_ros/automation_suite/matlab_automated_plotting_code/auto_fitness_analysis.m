%% Creates a scatter plot of the fitnesses for each generation and overlays a plot of average and best fitness
%
%
% GAS 11-8-17

%% Automation set up
fig = figure;
save_dir = strcat('~/simulation/ros_catkin_ws/src/evo_ros', plot_dir);
title_string = file_name(1:end-4);
title_string = strcat(title_string,' Generation vs Fitness');
title(title_string, 'Interpreter', 'none');
save_file_name = strcat(file_name(1:end-4),'_generation_vs_fitness.png')

%% Read in table and set up plotting arrays
cd('~/simulation/ros_catkin_ws/src/evo_ros/GA/logs');
table = readtable(file_name);
cd(save_dir)
hold on
avg = [];
best = [];
generation = [];

 %% Dynamically figure out population size and generation count
 A = table(table.Generation == 0, :);
 population_size = height(A);
 gen_count = round(height(table) / population_size);
 
%% loop through each generation creating a scattor plot
for i=0:gen_count-1
    
    % Create a table of just the individuals from this generation
    A = table(table.Generation == i, :);
    
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
h(1) = plot(generation,avg,'--r');
h(2) = plot(generation,best,'b');
legend(h, 'Average Fitness','Best Fitness', 'location', 'best')
xlabel('Generation')
ylabel('Fitness')
hold off

saveas(fig,save_file_name);
close(fig)