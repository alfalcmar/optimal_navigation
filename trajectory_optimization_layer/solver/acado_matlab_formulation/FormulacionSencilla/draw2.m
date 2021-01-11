figure(1);

subplot(3,1,1)
plot(STATES(:,1), STATES(:,2), 'LineWidth', 1.5); grid on; hold on;
plot([0 : time_interval : time_interval*(n_points-1)], points(:,1), '*r', 'LineWidth', 1.2); hold off;
title('X Position');
xlabel('Time [s]'); ylabel('X Position [m]');
legend('Drone trajectory', 'Desired points', 'Location', 'Best');

subplot(3,1,2)
plot(STATES(:,1), STATES(:,3), 'LineWidth', 1.5); grid on; hold on;
plot([0 : time_interval : time_interval*(n_points-1)], points(:,2), '*r', 'LineWidth', 1.2); hold off;
title('Y Position');
xlabel('Time [s]'); ylabel('Y Position [m]');
legend('Drone trajectory', 'Desired points', 'Location', 'Best');

subplot(3,1,3)
plot(STATES(:,1), STATES(:,4), 'LineWidth', 1.5); grid on; hold on;
plot([0 : time_interval : time_interval*(n_points-1)], points(:,3), '*r', 'LineWidth', 1.2); hold off;
title('Z Position');
xlabel('Time [s]'); ylabel('Z Position [m]');
legend('Drone trajectory', 'Desired points', 'Location', 'Best');


figure(2);
plot_circle(px_windmill, py_windmill, r_windmill); hold on;
plot(STATES(:,2), STATES(:,3), 'r', 'LineWidth', 1.5);
plot(points(:,1), points(:,2), 'g*', 'LineWidth', 1.2); hold off;
title('Windmill position and XY trajectory'); 
xlabel('X Position [m]'); ylabel('Y Position [m]');
legend('Windmill','XY trajectory', 'Desired Points', 'Location', 'Best'); grid on;


figure(3);

subplot(3,1,1)
plot(STATES(:,1), STATES(:,5), 'LineWidth', 1.5); grid;
title('X Velocity');
xlabel('Time [s]'); ylabel('X Velocity [m/s]');

subplot(3,1,2)
plot(STATES(:,1), STATES(:,6), 'LineWidth', 1.5); grid;
title('Y Velocity');
xlabel('Time [s]'); ylabel('Y Velocity [m/s]');

subplot(3,1,3)
plot(STATES(:,1), STATES(:,7), 'LineWidth', 1.5); grid;
title('Z Velocity');
xlabel('Time [s]'); ylabel('Z Velocity [m/s]');


figure(4);

subplot(3,1,1)
plot(CONTROLS(:,1), CONTROLS(:,2), 'LineWidth', 1.5); grid;
title('X Acceleration');
xlabel('Time [s]'); ylabel('X Acceleration [m/s²]');

subplot(3,1,2)
plot(CONTROLS(:,1), CONTROLS(:,3), 'LineWidth', 1.5); grid;
title('Y Acceleration');
xlabel('Time [s]'); ylabel('Y Acceleration [m/s²]');

subplot(3,1,3)
plot(CONTROLS(:,1), CONTROLS(:,4), 'LineWidth', 1.5); grid;
title('Z Acceleration');
xlabel('Time [s]'); ylabel('Z Acceleration [m/s²]');
% subplot(2,3,4)
% plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,5), 'r')
% title('Wheel Velocity [m/s]');
% 
% subplot(2,3,5)
% plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
% title('Damping Force [N]');
% 
% subplot(2,3,6)
% plot(disturbance(:,1), disturbance(:,2), 'r')
% title('Road Excitation [m]');