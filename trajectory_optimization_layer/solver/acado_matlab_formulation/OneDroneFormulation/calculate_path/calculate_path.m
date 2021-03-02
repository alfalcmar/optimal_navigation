function path = calculate_path (R, NO_FLY_ZONE_XY, pose_drone, desired_point, steps)

% First, obtain the current angle of the drone besides the no-fly zone:
% figure(1);
% plot_circle(NO_FLY_ZONE_XY(1), NO_FLY_ZONE_XY(2), R); hold on;
% plot(NO_FLY_ZONE_XY(1), NO_FLY_ZONE_XY(2), 'b*', 'LineWidth', 2);
% plot(pose_drone(1), pose_drone(2), 'r*', 'LineWidth', 2); hold off; grid;

% Position of the drone besides the no-fly zone:
Ax = pose_drone(1) - NO_FLY_ZONE_XY(1);
Ay = pose_drone(2) - NO_FLY_ZONE_XY(2);

% Position of the desired drone pose besides the no-fly zone:
Ax_d = desired_point(1) - NO_FLY_ZONE_XY(1);
Ay_d = desired_point(2) - NO_FLY_ZONE_XY(2);

% Calculate the angle:
current_phi = atan2(Ay,Ax);
if (current_phi < 0)
    current_phi = (pi + current_phi) + pi;
end

desired_phi = atan2(Ay_d,Ax_d);
if (desired_phi < 0)
    desired_phi = (pi + desired_phi) + pi;
end

% Give the steps for the path:
% steps = 30;

% Calculate the path:
phi     = zeros(steps, 1);
path    = zeros(steps, 3);

for i = 1:steps
    phi = ((desired_phi-current_phi)/steps) * i + current_phi;
    
    path(i, 1) = R*cos(phi) + NO_FLY_ZONE_XY(1); % x
    path(i, 2) = R*sin(phi) + NO_FLY_ZONE_XY(2); % y
    path(i, 3) = ((desired_point(3) - pose_drone(3))/steps) * i + pose_drone(3); % z
end

end