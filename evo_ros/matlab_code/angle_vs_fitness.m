%% Studying the fitness effect of the angle of the sonar
%
% Must be in the evo_ros directory
%
% GAS 10-30-17

%% Read in table and set up plotting arrays
cd('./GA/logs');
%file_name = 'single_sonar_evolution_40pop_60gen_run1.dat';
file_name = 'double_sonar_evolution_40pop_60gen_run1.dat';
title('Fitness vs Angle')
table = readtable(file_name);
cd('../../')
hold on
ang = [];
fitness = [];
fitness = [];
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
    
    for j=1:population_size
        
        if A.NumberOfSonar(1) == 1
            ang = [ang, A.S1_O(j)];
            fitness = [fitness, A.Fitness(j)];
        else
            ang = [ang, A.S1_O(j)];
            fitness = [fitness, A.Fitness(j)];
            ang = [ang, A.S2_O(j)];
            fitness = [fitness, A.Fitness(j)];
        end
        
    end

end

scatter(ang,fitness);
xlabel('Angle')
ylabel('Fitness')
hold off
