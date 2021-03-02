clc
clear all
close all

run("addpaths.m");

px_windmill = 12.0;
py_windmill = 16.0;
z_windmill_inspect = 5.0;
r_windmill  = 10.0;

initial_time = 0;
initial_point = [5, 5, 5];
end_point = [20, 18, 16];
end_time = 40;

points = 25; % + inicial

v_initial = [0, 0, 0];
a_initial = [0, 0, 0];

tray = zeros(points,4);

for i = 1 : points
    tray(i,1) = 0;
    tray(i,2) = i*(end_point(1)-initial_point(1))/points + initial_point(1);
    tray(i,3) = i*(end_point(2)-initial_point(2))/points + initial_point(2);
    tray(i,4) = i*(end_point(3)-initial_point(3))/points + initial_point(3);
end

% Se tienen que usar como mucho points-1 steps
out = formulation(initial_point, end_point, initial_time, end_time, points-1, px_windmill, py_windmill, r_windmill, tray, v_initial, a_initial, z_windmill_inspect);

estados = out.STATES

controles = out.CONTROLS

% difference between reference and real position
diferencia = tray(:,2:4) - estados(:,2:4)

muestra = 1;

% Los resultados parecen diferir de la solución real. En z, parece no
% moverse

if muestra == 1
    subplot(3,1,1);
    plot([0:(end_time/(points-1)):end_time], tray(:,2),'r'); grid on; hold on;
    plot(estados(:,1),estados(:,2),'b'); hold off;
    legend ('Reference path', 'Path done', 'Location', 'Best');
    xlabel('Time (s)'); ylabel('Position (m)'); title ('X position');
    
    subplot(3,1,2);
    plot([0:(end_time/(points-1)):end_time], tray(:,3),'r'); grid on; hold on;
    plot(estados(:,1),estados(:,3),'b'); hold off;
    legend ('Reference path', 'Path done', 'Location', 'Best');
    xlabel('Time (s)'); ylabel('Position (m)'); title ('Y position');
    
    subplot(3,1,3);
    plot([0:(end_time/(points-1)):end_time], tray(:,4),'r'); grid on; hold on;
    plot(estados(:,1),estados(:,4),'b'); hold off;
    legend ('Reference path', 'Path done', 'Location', 'Best');
    xlabel('Time (s)'); ylabel('Position (m)'); title ('Z position');
end