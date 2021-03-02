%% Plots
figure(1); set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(STATES(:,1), STATES(:,2), 'LineWidth', 1.2); grid on;
title('X Position');
xlabel('Time [s]'); ylabel('X Position [m]');

subplot(3,1,2)
plot(STATES(:,1), STATES(:,3), 'LineWidth', 1.2); grid on;
title('Y Position');
xlabel('Time [s]'); ylabel('Y Position [m]');

subplot(3,1,3)
plot(STATES(:,1), STATES(:,4), 'LineWidth', 1.2); grid on;
title('Z Position');
xlabel('Time [s]'); ylabel('Z Position [m]');



figure(2); set(gcf, 'Position', get(0, 'Screensize'));
j = 1;
for i = 1 : (size(STATES,1) - steps_rechor*steps_between_points-1)
    figure(2);
    
    k = j + steps_rechor*steps_between_points-1;
    if k >= size(STATES,1)
        k = size(STATES,1);
    end
    
    plot_circle(px_windmill, py_windmill, r_windmill); grid on; hold on;
    plot(STATES(j:k,2), STATES(j:k,3), 'r', 'LineWidth', 2);
    plot(STATES(j,2), STATES(j,3), 'r*', 'LineWidth', 3.5);
    plot(path(:,1), path(:,2), 'k*', 'LineWidth', 0.8);
    title('Windmill position and XY trajectory'); 
    xlabel('X Position [m]'); ylabel('Y Position [m]');
    legend('Windmill', 'XY trajectory', 'Start point', 'Desired Points', 'Location', 'Best'); hold off; grid on;
    
    j = k;
    
    pause(1);
    
    if k >= size(STATES,1)
        break;
    end
end



figure(3); set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(STATES(:,1), STATES(:,5), 'LineWidth', 1.5); grid on;
title('X Velocity');
xlabel('Time [s]'); ylabel('X Velocity [m/s]');

subplot(3,1,2)
plot(STATES(:,1), STATES(:,6), 'LineWidth', 1.5); grid on;
title('Y Velocity');
xlabel('Time [s]'); ylabel('Y Velocity [m/s]');

subplot(3,1,3)
plot(STATES(:,1), STATES(:,7), 'LineWidth', 1.5); grid;
title('Z Velocity');
xlabel('Time [s]'); ylabel('Z Velocity [m/s]');



figure(4); set(gcf, 'Position', get(0, 'Screensize'));

subplot(3,1,1)
plot(CONTROLS(:,1), CONTROLS(:,2), 'LineWidth', 1.5); grid on;
title('X Acceleration');
xlabel('Time [s]'); ylabel('X Acceleration [m/s²]');

subplot(3,1,2)
plot(CONTROLS(:,1), CONTROLS(:,3), 'LineWidth', 1.5); grid on;
title('Y Acceleration');
xlabel('Time [s]'); ylabel('Y Acceleration [m/s²]');

subplot(3,1,3)
plot(CONTROLS(:,1), CONTROLS(:,4), 'LineWidth', 1.5); grid on;
title('Z Acceleration');
xlabel('Time [s]'); ylabel('Z Acceleration [m/s²]');
