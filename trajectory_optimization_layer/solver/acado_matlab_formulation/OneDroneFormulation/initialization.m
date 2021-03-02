% Initializations

% Path
path = zeros((user_points - 1)*steps_path, 3);

% Initially, sim_time = 0
sim_time = 0;

% Initialize the variables where the data is going to be kept
STATES      = zeros (steps_between_points*steps_rechor*(size(path,1) - 1), 7);
CONTROLS    = zeros (steps_between_points*steps_rechor*(size(path,1) - 1), 4);

% Initialize j and k
j = 1;
k = steps_rechor+1;