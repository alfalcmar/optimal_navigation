%% Main script

clear all
clc

% 5 5 5 5 8 6 7 22 18 4 7 23.5 6 3.5 4.5 9 15 50 18
% 4 5 5 5 22 18 6 9 24.5 7 3 9 9 20 50 22
% 3 5 5 5 25 20 7 12 35 9 20 50 22

% It's necessary to include the paths of ACADO
run("addpaths.m");

% Need to know in case we try to reach a point inside the no-fly zone
px_windmill = 12.0;
py_windmill = 16.0;
r_windmill  = 6.0;

% Macros:
MIN_POINTS =  2;
MAX_POINTS = 20;


% Initializing variables
user_points     = 0;
time_interval   = 0;
steps           = 0;
steps_rec_hor   = 0;

display('Consider the first point as the initial one and the final point as the final one!');
display('Its important to know that there is a ');
display (['no-fly zone is on (x,y) = (', num2str(px_windmill), ', ', num2str(py_windmill), '), with radius = ', num2str(r_windmill), ' meters!']);
% user_points = input('Choose the number of points you want to go through (min 2, max 20): ');

while user_points < MIN_POINTS
    user_points = input(['Choose the number of points you want to go through (min ' num2str(MIN_POINTS) ', max ' num2str(MAX_POINTS) '): ']);
end

if user_points > MAX_POINTS
    user_points = MAX_POINTS;
end

points = zeros(user_points, 3);

% It is useful to plot the circle in order to input the desired points:
figure (1);
plot_circle(px_windmill, py_windmill, r_windmill);
xlabel('X axis [m]'); ylabel('Y axis [m]');

% Choose the points
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

% Close the plot
close 1;


display(' ');

while time_interval <= 0
    time_interval   = input('Desired time interval between the points (seconds): ');
end

while steps <= 0
    steps           = round(input('Number of steps between the points: '));
end

% So, the step-to-step time is:
step2step_time  = time_interval/steps;
total_steps     = (steps * (user_points-1) + 1);

display(' ');
display('On this simulation, receding horizon is considered, in function of steps');
display(['In order to choose the receding horizon, it is good to know that there will be ', num2str(total_steps), ' steps on this simulation']);

while steps_rec_hor <= 0
    steps_rec_hor   = input('Number of steps for recalculating (Receding Horizon): ');
end

% Execute the script "formulation.m" for each point:

% Initially, sim_time = 0
sim_time = 0;

% Initialize the variables where the data is going to be kept
STATES      = zeros (steps*(user_points-1) + 1, 8);
CONTROLS    = zeros (steps*(user_points-1) + 1, 5);
STATES_RH   = zeros (steps*(user_points-1) + 1, 8);
CONTROLS_RH = zeros (steps*(user_points-1) + 1, 5);






% Assuming that: 
%       - When it reaches a point, its acceleration will be 0 (not
%       necessarily its velocity)
%
%       - The path from the receding horizon is going to be get from the
%       whole path (including the whole points inserted by the command
%       window)
%      


% ------------ Main loop ------------ 
for i = 1 : (user_points - 1)
    
    % Let's get the initial velocity and acceleration (point to point)
    v_initial = [STATES(steps*(i-1) + 1, 5),    STATES(steps*(i-1) + 1, 6),     STATES(steps*(i-1) + 1, 7)];
    a_initial = [CONTROLS(steps*(i-1) + 1, 2),  CONTROLS(steps*(i-1) + 1, 3),   CONTROLS(steps*(i-1) + 1, 4)];
    
    % To know the path point to point (without receding horizon)
    out = formulation(points(i,:), points(i+1,:), sim_time, (sim_time + time_interval), steps, px_windmill, py_windmill, r_windmill,  v_initial, a_initial);
    
    % Refresh the sim_time
    sim_time = sim_time + time_interval;
    
    % Save the states (position and velocity) and the controls
    % (acceleration)
    STATES  ((steps*(i-1) + 1) : steps*i + 1, :) = out.STATES;
    CONTROLS((steps*(i-1) + 1) : steps*i + 1, :) = out.CONTROLS;
    
end
% ------------ End of main loop ------------





%%%%%%%%%%%%%%%%% RECEDING HORIZON %%%%%%%%%%%%%%%%%

% Reinitialize sim_time:
sim_time = 0;


% Initialize
steps_rec_hor2  = steps_rec_hor;
steps_left      = total_steps;

% ------------ Simulating the receding horizon loop ------------
for i = 1 : ceil(total_steps/steps_rec_hor2)
    
%     steps_left              = total_steps - i*steps_rec_hor2;
    
    if steps_left < steps_rec_hor2
        steps_rec_hor = steps_left - 1;
    else
        steps_left              = total_steps - i*steps_rec_hor2;
    end
        
    % If it's the first time running the simulation, use out (the point
    % to point states). If not, use the states generated by the
    % receding_horizon
        
    E = exist('out_RH','var');  % To make sure it doesn't run more than
    % once (in case we operate with a inner loop)
    
    
    if (i == 1 && E == 0)       % If it is the first time simulating:
            
        % Get the initial and final position, velocity and acceleration (receding horizon)
        p_initial = [STATES(steps_rec_hor2*(i-1) + 1, 2),    STATES(steps_rec_hor2*(i-1) + 1, 3),     STATES(steps_rec_hor2*(i-1) + 1, 4)];
        v_initial = [STATES(steps_rec_hor2*(i-1) + 1, 5),    STATES(steps_rec_hor2*(i-1) + 1, 6),     STATES(steps_rec_hor2*(i-1) + 1, 7)];
        a_initial = [CONTROLS(steps_rec_hor2*(i-1) + 1, 2),  CONTROLS(steps_rec_hor2*(i-1) + 1, 3),   CONTROLS(steps_rec_hor2*(i-1) + 1, 4)];
        
        p_final = [STATES(steps_rec_hor2*i + 1, 2),    STATES(steps_rec_hor2*i + 1, 3),     STATES(steps_rec_hor2*i + 1, 4)];
        v_final = [STATES(steps_rec_hor2*i + 1, 5),    STATES(steps_rec_hor2*i + 1, 6),     STATES(steps_rec_hor2*i + 1, 7)];
        a_final = [CONTROLS(steps_rec_hor2*i + 1, 2),  CONTROLS(steps_rec_hor2*i + 1, 3),   CONTROLS(steps_rec_hor2*i + 1, 4)];        
        
    else if (i > 1)             % If it has been simulated more than once:
            
            % Get the initial and final position, velocity and acceleration (receding horizon)
            p_initial = [STATES_RH(steps_rec_hor2*(i-1) + 1, 2),    STATES_RH(steps_rec_hor2*(i-1) + 1, 3),     STATES_RH(steps_rec_hor2*(i-1) + 1, 4)];
            v_initial = [STATES_RH(steps_rec_hor2*(i-1) + 1, 5),    STATES_RH(steps_rec_hor2*(i-1) + 1, 6),     STATES_RH(steps_rec_hor2*(i-1) + 1, 7)];
            a_initial = [CONTROLS_RH(steps_rec_hor2*(i-1) + 1, 2),  CONTROLS_RH(steps_rec_hor2*(i-1) + 1, 3),   CONTROLS_RH(steps_rec_hor2*(i-1) + 1, 4)];
        
            p_final = [STATES((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 2),    STATES((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 3),     STATES((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 4)];
            v_final = [STATES((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 5),    STATES((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 6),     STATES((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 7)];
            a_final = [CONTROLS((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 2),  CONTROLS((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 3),   CONTROLS((steps_rec_hor2*(i-1) + steps_rec_hor + 1), 4)];
            
            
        
        end
    end
    
    out_RH = formulation_RH(p_initial, p_final, sim_time, (sim_time + step2step_time*steps_rec_hor), steps_rec_hor, px_windmill, py_windmill, r_windmill, v_initial, a_initial, a_final);
    
    % Refresh the sim_time
    sim_time = sim_time + step2step_time*steps_rec_hor2;
        
        
    
    % Save the states (position and velocity) and the controls
    % (acceleration)
    STATES_RH   ((steps_rec_hor2*(i-1) + 1) : (steps_rec_hor2*(i-1) + steps_rec_hor + 1), :) = out_RH.STATES;
    CONTROLS_RH ((steps_rec_hor2*(i-1) + 1) : (steps_rec_hor2*(i-1) + steps_rec_hor + 1), :) = out_RH.CONTROLS;

end
% ------------ End of receding horizon loop ------------ 




% Plot the results
draw3;