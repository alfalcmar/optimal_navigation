%% Main script %%

clear all
close all
clc

warning('off');

% It's necessary to include the paths of ACADO
run("addpaths.m");

% Initialize the parameters necessary for the simulation
run("param.m");

% Calculate the path
path = get_full_path(r_windmill, px_windmill, py_windmill, points, steps_path, user_points);

% ------------ Main loop ------------ 
% Assuming that the drone is already on the cylinder 
while (j < size(path,1))

    % Get the initial velocity and acceleration (point to point)

    v_initial = [  STATES(steps_iteration*(j-1) + 1, 5),    STATES(steps_iteration*(j-1) + 1, 6),     STATES(steps_iteration*(j-1) + 1, 7)]
    a_initial = [CONTROLS(steps_iteration*(j-1) + 1, 2),  CONTROLS(steps_iteration*(j-1) + 1, 3),   CONTROLS(steps_iteration*(j-1) + 1, 4)]
    
    % Following points
    if ( (k-1+steps_rechor) < size(path,1) )
        if j == 1
            initial_point = path(1,:);
            next_points_(1 : steps_rechor+1, :) = path(1:1+steps_rechor, :);
        else
            initial_point = [  STATES(steps_iteration*(j-1) + 1, 2),    STATES(steps_iteration*(j-1) + 1, 3),     STATES(steps_iteration*(j-1) + 1, 4)];
            next_points_(1 : steps_rechor+1, :) = path(k-1 : k-1+steps_rechor, :);
        end
    else
        initial_point = [  STATES(steps_iteration*(j-1) + 1, 2),    STATES(steps_iteration*(j-1) + 1, 3),     STATES(steps_iteration*(j-1) + 1, 4)];
        next_points_(1 : (size(path,1) - k),:) = path(k+1:size(path,1),:);
    end

    % Calculate the next_points (considering the steps_between_points)
    next_points(:,2:4) = calculate_path_for_rh(r_windmill, [px_windmill, py_windmill], next_points_(:,:), steps_between_points);
    
    % Following the trajectory with straight lines
    out = formulation(initial_point, sim_time, (sim_time + time_interval*(steps_rechor/steps_path)), steps_rechor*steps_between_points-1, next_points, v_initial, a_initial);
    
    % Refresh the sim_time
    sim_time = sim_time + time_interval*(steps_rechor/steps_path);

    % Save the states (position and velocity) and the controls
    % (acceleration)
    STATES  (steps_iteration*(j-1) + 1 : steps_iteration*j + 1, :) = out.STATES;
    CONTROLS(steps_iteration*(j-1) + 1 : steps_iteration*j + 1, :) = out.CONTROLS;

    % Refresh j and k
    j = j + 1;
    if k >= size(path,1)
        break;
    else
        k = k + steps_rechor;
    end
end

% In case that the variables have a zero:
k = 2;
while (STATES(k,1) ~= 0.0 && k < size(STATES,1))
    k = k + 1;
end
aux = zeros(k-1,7);
auxC = zeros(k-1,4);
aux(1:k-1,:)  = STATES(1:k-1,:);
auxC(1:k-1,:) = CONTROLS(1:k-1,:);

STATES                   = zeros(k-1,7);
CONTROLS                 = zeros(k-1,4);
STATES(1:k-1,:)          = aux(1:k-1,:);
CONTROLS(1:k-1,:)        = auxC(1:k-1,:);


% Plot the results
draw3;