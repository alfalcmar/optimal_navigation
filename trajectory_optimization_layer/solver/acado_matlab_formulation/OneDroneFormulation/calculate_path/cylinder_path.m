% Script for cylinder-form paths:

% Cylinder coords: (rho, phi, z)
% If rho is constant --> vertical cylinder
% If phi constant --> vertical semiplane
% If z constant --> horizontal plane

% In this new formulation, we want to have a constant distance between the
% drones and the no-fly zone.

% Parameters given:
R     = 25; % radius [m]
rho   = R;  % rho = radius (constant along the path)

% phi   = 0;
% z     = 2;

% Desired points to reach (on cartesian coords):
% TO DO

% No-fly zone's parameters:
NO_FLY_ZONE_R   = 15;      % No-fly zone radius [m] probably not necessary over the last formulation
NO_FLY_ZONE_XY  = [26, 18];

% Start position of the drone:
pose_drone = [-2, -2, 3];  % [x, y, z] [m]

% Desired point
desired_point = [20, 60, 8];

path   = calculate_path (R, NO_FLY_ZONE_XY, pose_drone, desired_point);
draw;