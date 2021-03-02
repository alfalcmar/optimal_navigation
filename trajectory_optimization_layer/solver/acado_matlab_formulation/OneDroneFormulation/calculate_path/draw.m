figure(1);

plot_circle(NO_FLY_ZONE_XY(1), NO_FLY_ZONE_XY(2), R); hold on;
plot(NO_FLY_ZONE_XY(1), NO_FLY_ZONE_XY(2), 'b*', 'LineWidth', 2);
plot(pose_drone(1), pose_drone(2), 'r*', 'LineWidth', 2);
plot(desired_point(1), desired_point(2), 'k*', 'LineWidth', 2);
plot(path(:,1), path(:,2), 'g-', 'LineWidth', 1.5); hold off; grid on;
xlabel('X [m]'); ylabel('Y [m]'); title('XY path');
legend('No-fly zone circle', 'No-fly zone center', 'Drone initial pose', 'Drone desired pose', 'Path', 'Location', 'Best');


% To draw the cylinder:
[x, y, z] = cylinder(R);
x(:)     = x(:) + NO_FLY_ZONE_XY(1);
y(:)     = y(:) + NO_FLY_ZONE_XY(2);
z(2,:)   = z(2,:)*20;

figure(2);
plot3(pose_drone(1), pose_drone(2), pose_drone(3), 'b*', 'LineWidth', 2); hold on;
plot3(desired_point(1), desired_point(2), desired_point(3), 'r*', 'LineWidth', 2);
plot3(path(:,1), path(:,2), path(:,3), 'g-*', 'LineWidth', 1.5);
surf(x,y,z); hold off; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z[m]'); title('XYZ path');
legend('Drone initial pose', 'Drone desired pose', 'Path', 'No-fly zone', 'Location', 'Best');
