% Choose a path from 0 to 3:
path_ = 1;

% Read the points of the desired path from param_path.m
run("param_path.m");

% Need to know in case we try to reach a point inside the no-fly zone
px_windmill = 12.0;
py_windmill = 16.0;
r_windmill  = 15.0;

% Number of step points that the path has. In total,
% (user_points-1)*steps_path waypoints
steps_path   = 15;

% Receding horizon: on each step, the drone goes through steps_rechor
% waypoints of the path
steps_rechor = 3;

% Between the waypoints of the path, it will have x steps_between_points
steps_between_points = 15;

% Steps done every iteration
steps_iteration = steps_between_points*steps_rechor - 1;

% Get the initializations
run("initialization.m");