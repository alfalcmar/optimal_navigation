figure(1); set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(STATES(:,1), STATES(:,2), 'LineWidth', 1.2); grid on; hold on;
plot(STATES_RH(:,1), STATES_RH(:,2), 'y', 'LineWidth', 1.5);
plot([0 : time_interval : time_interval*(n_points-1)], points(:,1), '*r', 'LineWidth', 1.2); hold off;
title('X Position');
xlabel('Time [s]'); ylabel('X Position [m]');
legend('Drone trajectory (without RH)', 'Drone trajectory (with RH)', 'Desired points', 'Location', 'Best');

subplot(3,1,2)
plot(STATES(:,1), STATES(:,3), 'LineWidth', 1.2); grid on; hold on;
plot(STATES_RH(:,1), STATES_RH(:,3), 'y','LineWidth', 1.5);
plot([0 : time_interval : time_interval*(n_points-1)], points(:,2), '*r', 'LineWidth', 1.2); hold off;
title('Y Position');
xlabel('Time [s]'); ylabel('Y Position [m]');
legend('Drone trajectory (without RH)', 'Drone trajectory (with RH)', 'Desired points', 'Location', 'Best');

subplot(3,1,3)
plot(STATES(:,1), STATES(:,4), 'LineWidth', 1.2); grid on; hold on;
plot(STATES_RH(:,1), STATES_RH(:,4), 'y','LineWidth', 1.5);
plot([0 : time_interval : time_interval*(n_points-1)], points(:,3), '*r', 'LineWidth', 1.2); hold off;
title('Z Position');
xlabel('Time [s]'); ylabel('Z Position [m]');
legend('Drone trajectory (without RH)', 'Drone trajectory (with RH)', 'Desired points', 'Location', 'Best');



figure(2); set(gcf, 'Position', get(0, 'Screensize'));

plot_circle(px_windmill, py_windmill, r_windmill); hold on;
plot(STATES(:,2), STATES(:,3), 'r', 'LineWidth', 1.5);
plot(STATES_RH(:,2), STATES_RH(:,3), 'y', 'LineWidth', 1.5);
plot(points(:,1), points(:,2), 'k*', 'LineWidth', 1.5); hold off;
title('Windmill position and XY trajectory'); 
xlabel('X Position [m]'); ylabel('Y Position [m]');
legend('Windmill', 'XY trajectory (without RH)', 'XY trajectory (with RH)', 'Desired Points', 'Location', 'Best'); grid on;



figure(3); set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(STATES(:,1), STATES(:,5), 'LineWidth', 1.5); grid on; hold on;
plot(STATES_RH(:,1), STATES_RH(:,5), 'r', 'LineWidth', 1.5); hold off;
title('X Velocity');
xlabel('Time [s]'); ylabel('X Velocity [m/s]');
legend('Without RH', 'With RH', 'Location', 'Best');

subplot(3,1,2)
plot(STATES(:,1), STATES(:,6), 'LineWidth', 1.5); grid on; hold on;
plot(STATES_RH(:,1), STATES_RH(:,6), 'r', 'LineWidth', 1.5); hold off;
title('Y Velocity');
xlabel('Time [s]'); ylabel('Y Velocity [m/s]');
legend('Without RH', 'With RH', 'Location', 'Best');

subplot(3,1,3)
plot(STATES(:,1), STATES(:,7), 'LineWidth', 1.5); grid; hold on;
plot(STATES_RH(:,1), STATES_RH(:,7), 'r', 'LineWidth', 1.5); hold off;
title('Z Velocity');
xlabel('Time [s]'); ylabel('Z Velocity [m/s]');
legend('Without RH', 'With RH', 'Location', 'Best');



figure(4); set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(CONTROLS(:,1), CONTROLS(:,2), 'LineWidth', 1.5); grid on; hold on;
plot(CONTROLS_RH(:,1), CONTROLS_RH(:,2), 'r', 'LineWidth', 1.5); hold off;
title('X Acceleration');
xlabel('Time [s]'); ylabel('X Acceleration [m/s²]');
legend('Without RH', 'With RH', 'Location', 'Best');

subplot(3,1,2)
plot(CONTROLS(:,1), CONTROLS(:,3), 'LineWidth', 1.5); grid on; hold on;
plot(CONTROLS_RH(:,1), CONTROLS_RH(:,3), 'r', 'LineWidth', 1.5); hold off;
title('Y Acceleration');
xlabel('Time [s]'); ylabel('Y Acceleration [m/s²]');
legend('Without RH', 'With RH', 'Location', 'Best');

subplot(3,1,3)
plot(CONTROLS(:,1), CONTROLS(:,4), 'LineWidth', 1.5); grid on; hold on;
plot(CONTROLS_RH(:,1), CONTROLS_RH(:,4), 'r', 'LineWidth', 1.5); hold off;
title('Z Acceleration');
xlabel('Time [s]'); ylabel('Z Acceleration [m/s²]');
legend('Without RH', 'With RH', 'Location', 'Best');