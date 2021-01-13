% Choose a path from 0 to 3:
path = 1;

% Read the params from param_path.m
run("param_path.m");

% Need to know in case we try to reach a point inside the no-fly zone
px_windmill = 12.0;
py_windmill = 16.0;
z_windmill_inspect = 5.0;
r_windmill  = 6.0;

% Initial z position of the drone [m]:
z_start_drone = 3;

% So, the step-to-step time is:
step2step_time  = time_interval/steps;
total_steps     = (steps * (user_points-1) + 1);