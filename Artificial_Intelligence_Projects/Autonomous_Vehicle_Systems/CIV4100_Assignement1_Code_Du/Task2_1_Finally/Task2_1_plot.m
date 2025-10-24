
% Task2_1 Plot

function Task2_1_plot()

% === Load the inflated map to show the obstacle area ===
load('task1_1_map.mat');
inflatedMap = copy(map);
inflate(inflatedMap, 1.1);

% === Call Task2_1_ref.m to obtain the variables of the reference trajectory ===
dt = 0.1;
[x_ref, y_ref, theta_ref, s_ref, v_ref, a_ref, omega_ref, alpha_ref, t_ref] = Task2_1_ref(dt);

% Plot the trajectory with map background
figure;
show(inflatedMap); hold on;        % Display map background
title('Smooth trajectory avoiding obstacles');
xlabel('x (m)'); ylabel('y (m)');
set(gca, 'FontSize', 12);
grid on;

% Draw the trajectory curve (blue line)
plot(x_ref, y_ref, 'b-', 'LineWidth', 1.5);

% Draw the heading arrows (red arrows, every 5 points)
arrow_interval = 5;
quiver(x_ref(1:arrow_interval:end), y_ref(1:arrow_interval:end), ...
       cos(theta_ref(1:arrow_interval:end)), sin(theta_ref(1:arrow_interval:end)), ...
       0.8, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);

% Must-pass waypoints (black cross)
wp_must = [0.2  5   20  20   8   8   13.2;
             3   1    3   22  22  16   16];
plot(wp_must(1,:), wp_must(2,:), 'kx', 'MarkerSize', 9, 'LineWidth', 2);

legend({'Trajectory', 'Heading', 'Must-pass Waypoints'}, 'Location', 'northeastoutside');

% Plot x(t) and y(t) position versus time
figure;
plot(t_ref, x_ref, 'b', 'LineWidth', 2); hold on;
plot(t_ref, y_ref, 'r', 'LineWidth', 2);
title('x(t) and y(t) vs Time');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x(t)', 'y(t)');
set(gca, 'FontSize', 12);
grid on;

% Plot arc length, velocity, and acceleration versus time
figure;
set(gcf, 'Position', [100 100 600 900]); % Set figure size nicely
subplot(3,1,1);
plot(t_ref, s_ref, 'g', 'LineWidth', 1.5);
ylabel('s(t) (m)');
title('Arc Length Profile');
set(gca, 'FontSize', 12);
grid on;

subplot(3,1,2);
plot(t_ref, v_ref, 'k', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
title('Velocity Profile');
set(gca, 'FontSize', 12);
grid on;

subplot(3,1,3);
plot(t_ref, a_ref, 'm', 'LineWidth', 1.5);
ylabel('Acceleration (m/s^2)');
xlabel('Time (s)');
title('Acceleration Profile');
set(gca, 'FontSize', 12);
grid on;

end





