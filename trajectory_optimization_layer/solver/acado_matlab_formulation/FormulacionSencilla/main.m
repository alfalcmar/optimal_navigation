%% Main script

clear all
clc

% It's necessary to include the paths of ACADO
run("addpaths.m");

% Need to know in case we try to reach a point inside the no-fly zone
px_windmill = 12.0;
py_windmill = 16.0;
r_windmill  = 3.0;

% Macros:
MAX_POINTS = 20;

display('Consider the first point as the initial one and the final point as the final one!');
display('Its important to know that there is a ');
display (['no-fly zone is on (x,y) = (', num2str(px_windmill), ', ', num2str(py_windmill), '), with radius = ', num2str(r_windmill), ' meters!']);
user_points = input('Choose the number of points you want to go through (min 2, max 20): ');

if user_points < 2
    while user_points < 2
        user_points = input('Choose the number of points you want to go through (min 2, max 20): ');
    end
end

if user_points > MAX_POINTS
    user_points = MAX_POINTS;
end

points = zeros(user_points, 3);

for i = 1 : user_points
    points(i,1) = input(['Point ' num2str(i)  ' X: ']);
    points(i,2) = input(['Point ' num2str(i)  ' Y: ']);
    points(i,3) = input(['Point ' num2str(i)  ' Z: ']);

    if ( ( (points(i,1) - px_windmill)^2 + (points(i,2) - py_windmill)^2 ) <= r_windmill^2 )
        
        while ( ( (points(i,1) - px_windmill)^2 + (points(i,2) - py_windmill)^2 ) <= r_windmill^2 )
            display ('You are trying to pass over the no-fly zone!');
            display (['Remember, the no-fly zone is on (x,y) = (', num2str(px_windmill), ', ', num2str(py_windmill), '), with radius = ', num2str(r_windmill), ' meters!']);
            display (' ');
            
            points(i,1) = input(['Point ' num2str(i)  ' X: ']);
            points(i,2) = input(['Point ' num2str(i)  ' Y: ']);
            points(i,3) = input(['Point ' num2str(i)  ' Z: ']);
        end
    end
    n_points = i;
end

display(' ');

time_interval   = input('Desired time interval between the points (seconds): ');
steps           = input('Number of steps between the points: ');

% Execute the script "formulation.m" for each point:

% Initially, sim_time = 0
sim_time = 0;

% Initialize the variables where the data is going to be kept
STATES   = zeros (steps*(user_points-1) + 1, 7);
CONTROLS = zeros (steps*(user_points-1) + 1, 4);

for i = 1 : (user_points - 1)
    
    % Let's get the initial velocity and acceleration
    v_initial = [STATES(steps*(i-1) + 1, 5),    STATES(steps*(i-1) + 1, 6),     STATES(steps*(i-1) + 1, 7)];
    a_initial = [CONTROLS(steps*(i-1) + 1, 2),  CONTROLS(steps*(i-1) + 1, 3),   CONTROLS(steps*(i-1) + 1, 4)];
    
    out = formulation(points(i,:), points(i+1,:), sim_time, (sim_time + time_interval), steps, px_windmill, py_windmill, r_windmill,  v_initial, a_initial);
    
    % Refresh the sim_time
    sim_time = sim_time + time_interval;
    
    % Save the states (position and velocity) and the controls
    % (acceleration)
    STATES  ((steps*(i-1) + 1) : steps*i + 1, :) = out.STATES;
    CONTROLS((steps*(i-1) + 1) : steps*i + 1, :) = out.CONTROLS;
end


draw2;