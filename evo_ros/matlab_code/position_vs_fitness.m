%% Studying the fitness effect of the position of the sonars
%
% Must be in the evo_ros directory
%
% GAS 11-7-17

%% Read in table and set up plotting arrays
cd('./GA/logs');
%file_name = 'single_sonar_evolution_40pop_60gen_run1.dat';
file_name = 'double_sonar_evolution_40pop_60gen_run1.dat';
table = readtable(file_name);
cd('../../')
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

figure;
X1 = y1;
Y1 = x1;
Z = fitness;
s(1) = stem3(X1,Y1,Z, 'blue');
hold on;
X2 = y2;
Y2 = x2;
s(2) = stem3(X2,Y2,Z,'red');

legend(s([1,2]),{'Sensor 1', 'Sensor 2'})
title('Sensor Position vs Fitness')
xlabel('Left / Right from center (meters)')
ylabel('Forward / Back from center (meters)')
zlabel('Fitness')
hold off
