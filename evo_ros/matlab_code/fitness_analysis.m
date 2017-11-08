%% Creates a scatter plot of the fitnesses for each generation and overlays a plot of average and best fitness
%
% Must be in the evo_ros directory
%
% GAS 10-30-17

%% Read in table and set up plotting arrays
cd('./GA/logs');
file_name = 'single_sonar_evolution_40pop_60gen_run1.dat';
file_name = 'double_sonar_with_knockout_run1.dat';
title('Fitness vs Generation')
table = readtable(file_name);
cd('../../')
hold on
avg = [];
best = [];
generation = [];

 %% Dynamically figure out population size and generation count
 A = table(table.Generation == 0, :);
 population_size = height(A);
 gen_count = height(table) / population_size;
 
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
plot(generation,avg,'r')
plot(generation,best,'b')
legend('Average Fitness','Best Fitness')
xlabel('Generation')
ylabel('Fitness')
hold off
