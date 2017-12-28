%% Creates a block diagram plot of the rover and the different regions that the sensors can spawn in
% It then imports the winning individuals from the sensor placement runs
% and overlays the sensors placements onto the diagram to represent the
% sensor placements
%
% GAS 2017-12-26

% Read in winning indivuals from logged file
winning_inds = readtable('./logs_of_interest/best_individuals.txt');

% Define list of sensor colors
colors = [([115 120 72] ./ 255); ([11 104 127] ./ 255); ([124 3 58] ./ 255);
    ([58 99 2] ./ 255); ([231 25 12] ./ 255); ([23 149 70] ./ 255); 
    ([18 9 229] ./ 255); ([213 32 203] ./ 255); ([105 156 17] ./ 255); 
    ([15 154 191] ./ 255);]; 


% Iterate through the individuals in the log file
for i=1:height(winning_inds)
% for i=7:7  
    ind = winning_inds(i,:);
     
    diagram = figure;
    plot_rover_diagram(ind{1,2} , ind{1,1});
    
    % Iterate through the sensors on this ind
    for j=4:3:31
        color = colors((j-1)/3,:);
        ind_angle = ind{1, j} * -1; % -1 because gazebo flips orienation
        ind_pos_x = ind{1,j+1} * 100; % times 100 to conver meters (gazebo) to cm (shown on plot)
        ind_pos_y = ind{1,j+2} * -100; % negative again to deal with Gazebos flip
        overlay_sensor(ind_pos_y, ind_pos_x,ind_angle, color); % swapping x and y values because Gazebo flips these axis

    end
end


function b = overlay_sensor(pos_x, pos_y, angle, color)
    hold on;
    
    
    sensor_color = color;
    vision_cone_color = color;
    viewing_ang = 15;
    mid_line_length = 7.5;
    cone_lines_length = 20;
    
    % Marker for the sensor
    plot(pos_x, pos_y,...
        'r.',...
        'color', sensor_color,...
        'MarkerSize', 30)
    
    % Line indicating the direction that the sensor is facing
    plot([pos_x (mid_line_length * sind(angle)+pos_x)],...
         [pos_y (mid_line_length * cosd(angle)+pos_y)],...
         'color', sensor_color)
     
    % Left line of the vision cone
    plot([pos_x (cone_lines_length * sind(angle - viewing_ang)+pos_x)],...
         [pos_y (cone_lines_length * cosd(angle - viewing_ang)+pos_y)],...
         'color', vision_cone_color)
     
     % Left line of the vision cone
    plot([pos_x (cone_lines_length * sind(angle + viewing_ang)+pos_x)],...
         [pos_y (cone_lines_length * cosd(angle + viewing_ang)+pos_y)],...
         'color', vision_cone_color)
    

end

function a = plot_rover_diagram(ind_name, ind_fitness)
    hold on;
    
    pw = 5; % placement width - width of outside ledge to place sensor on 
    % Outside border
    rectangle('Position',[-15 -25 30 50]) % start x,y , width, height
    
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
    
    % Overlay individuals name
    text(0,-10,ind_name, 'Interpreter', 'none',...
        'HorizontalAlignment', 'center',...
        'BackgroundColor','white')
    
    % Label front of rover
    text(0,27,'Front', 'Interpreter', 'none',...
        'HorizontalAlignment', 'center')
    
    % Label back of rover
    text(0,-27,'Back', 'Interpreter', 'none',...
        'HorizontalAlignment', 'center')
    
    % Overlay Fitness value
    text(22.5, -15, 'Fitness:',...
        'Interpreter', 'none',...
        'HorizontalAlignment', 'center')
    text(22.5, -18, string(ind_fitness),...
        'Interpreter', 'none',...
        'HorizontalAlignment', 'center')
    
    
    axis([-30 30 -30 40])
    
    
end

