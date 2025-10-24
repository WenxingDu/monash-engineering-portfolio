% batch_2.m 
% ===============================================================
% This part performs dynamic tracking of the reference trajectory generated in Task 2.1 using Receding Horizon Planning
% In each iteration, a local segment of the reference trajectory is extracted, and `fmincon` is called to solve a finite-horizon optimization problem
% The system state is then updated and the optimized segment is appended to the overall trajectory 
% This process continues until the reference trajectory is exhausted


clear; clc;

% Parameters 
N  = 32;                     % Number of discretisation intervals
Nx = 5;                      % Number of state variables
Nu = 2;                      % Number of input variables
T  = 5;                      % Seconds
dt = T / N;                  % Step size

xg = [13.2; 16];              % Goal position (x, y)
d_goal = 0.5;                 % Stopping distance threshold

% Load full reference trajectory
% The initial state is taken from the first point of the reference trajectory 
% and supplemented with the initial cost (which is 0)

[x_ref_full, y_ref_full, theta_ref_full, s_ref_full, v_ref_full, a_ref_full, omega_ref_full, alpha_ref_full, t_ref_full] = Task2_2_ref(dt);

x0 = [x_ref_full(1); y_ref_full(1); theta_ref_full(1); v_ref_full(1); 0];

% Containers for full trajectory 
x_all = [];
u_all = [];
t_all = [];
t0 = 0;

% Fixed problem info 
prob_info_fixed = struct('T', T, 'N', N, 'Nu', Nu, 'Nx', Nx, ...
                   'wp', 0.1, 'wh', 1, ...
                   'centres', [3.2 14.0; 3.2 20.0; 14.2 8.0; 14.2 20.0], 'radii', 3);

% Set variables to record the trajectory of the entire running process, 
% limit the steering and acceleration range of each step, and splice
lb_control = -inf(Nu,N); ub_control = inf(Nu,N);
lb_state = -inf(Nx,N);   ub_state = inf(Nx,N);
lb_control(1,:) = -0.5;  ub_control(1,:) = 0.5;
lb_control(2,:) = -3;    ub_control(2,:) = 2;

lb = [-inf((N+1)*Nx,1); lb_control(:)];
ub = [ inf((N+1)*Nx,1); ub_control(:)];


% Controlling the Behavior of fmincon
% Constraints do not have to be exactly 0
options = optimoptions('fmincon', ...
    'Display', 'iter-detailed', ...
    'MaxIterations', 300, ...
    'MaxFunctionEvaluations', 1e5, ...
    'ConstraintTolerance', 5e-2, ...
    'OptimalityTolerance', 1e-2);          
    % The setting is not very small in order to make the optimization process end faster

% Give fmincon a reasonable guess in advance to speed up convergence
init_control = repmat([0.01; 0.5], 1, N);

% ======== Main RHP Loop ==========
max_iter = 100;
iter = 0;

tic;
% End condition: the distance between the current car position and the target position is less than the threshold
while (x0(1) - xg(1))^2 + (x0(2) - xg(2))^2 > d_goal^2 && iter < max_iter
    iter = iter + 1;
    % Displays the current number of iterations and the Euclidean distance from the target point. This is convenient for debugging
    dist2goal = norm(x0(1:2) - xg);
    fprintf("Iteration %d, distance to goal: %.4f\n", iter, dist2goal);  

    % The starting index and ending index of the current optimization window
    idx_start = max(1, round(t0/dt) + 1);
    idx_end = min(length(t_ref_full), idx_start + N);

    % Determine whether the reference trajectory has been reached
    if idx_start >= length(x_ref_full) - 1
        disp('Reference trajectory exhausted. Exiting loop.');
        break;
    end

    fprintf("Ref segment index: %d ~ %d / %d\n", idx_start, idx_end, length(x_ref_full));  
    
    % Extract the reference path segment within the current time window from the complete reference trajectory 
    % for this round of optimization
    x_rhp = x_ref_full(idx_start:idx_end);
    y_rhp = y_ref_full(idx_start:idx_end);
    theta_rhp = theta_ref_full(idx_start:idx_end);

    
    % And padding is performed on the insufficient length to ensure that there are N+1 steps
    if isempty(x_rhp)
        x_rhp = zeros(1, N+1);
        y_rhp = zeros(1, N+1);
        theta_rhp = zeros(1, N+1);
    elseif length(x_rhp) < N+1
        x_rhp(end+1:N+1) = x_rhp(end);
        y_rhp(end+1:N+1) = y_rhp(end);
        theta_rhp(end+1:N+1) = theta_rhp(end);
    end



    % prob_info needs to be updated every round because the starting point x0 will change 
    % and the reference trajectory x_ref also needs to use local segments
    prob_info = prob_info_fixed;
    prob_info.x0 = x0;
    prob_info.x_ref = [x_rhp; y_rhp; theta_rhp];

    % Initial guess trajectory
    % Use Euler's method to integrate the system state, generate the initial state trajectory, and then superimpose it with the initial 
    % control to form the optimization variable
    init_state = zeros(Nx, N+1);
    init_state(:,1) = x0;
    for idx = 2:N+1
        init_state(:,idx) = init_state(:,idx-1) + dt * sys_h_2(init_state(:,idx-1), init_control(:,min(idx-1,N)), prob_info);
    end
    eta0 = [reshape(init_state, [], 1); reshape(init_control, [], 1)];

    % Solve optimization
    % Output the optimal solution eta_star, from which the state x_star and control u_star can be extracted
    fun     = @(eta) cost_2(eta, prob_info);
    nonlcon = @(eta) nconst_2(eta, prob_info);

    [eta_star, cost_star, exitflag, output] = fmincon(fun, eta0, [], [], [], [], lb, ub, nonlcon, options);
     
    % A mechanism to detect whether the current optimization is successful. If the optimization fails (for example, 
    % the state or input does not satisfy the constraints), the entire RHP cycle is aborted.
    fprintf("Exit flag: %d\n", exitflag);
    disp(output.message)

    if exitflag <= 0
        warning('fmincon failed. Exiting loop early.');
        break;
    end

    % Parse solution
    x_star = reshape(eta_star(1:(N+1)*Nx), Nx, N+1);
    u_star = reshape(eta_star((N+1)*Nx+1:end), Nu, N);

    % Only save the trajectory and control results of the first half
    % Re-optimize the next paragraph
    K = round(N/2);
    x_all = [x_all, x_star(:,2:K+1)];
    u_all = [u_all, u_star(:,1:K)];
    t_all = [t_all, t0 + (1:K)*dt];

    % Update state
    x0 = x_star(:,K+1);
    t0 = t0 + K*dt;
end

total_time = toc;
fprintf("Total RHP planning time: %.2f seconds\n", total_time)

% === Final Visualization ===

figure('Name','RHP Trajectory Summary','Position',[100 100 1200 800])

subplot(2,2,1)
% Set up a good color
plot(x_all(1,:), x_all(2,:), 'Color', [0.4 0.6 1], 'LineWidth', 2);
hold on
plot(x_ref_full, y_ref_full, 'r--')
viscircles(prob_info_fixed.centres, prob_info_fixed.radii * ones(size(prob_info_fixed.centres,1),1));
legend('Optimized Trajectory', 'Reference Trajectory')
axis equal, xlabel('x (m)'), ylabel('y (m)'), title('Trajectory Tracking')

grid on

subplot(2,2,2)
plot(rad2deg(x_all(3,:)))
xlabel('Step'), ylabel('Heading (deg)'), title('Vehicle Heading')
grid on

subplot(2,2,3)
plot(x_all(4,:))
xlabel('Step'), ylabel('Speed (m/s)'), title('Forward Speed')
grid on

subplot(2,2,4)
plot(u_all(1,:), 'b')
hold on
plot(u_all(2,:), 'r')
legend('Steering Input', 'Acceleration')
xlabel('Step'), title('Control Inputs')
grid on


% ===== Complete RHP Trajectory over Parkinglot Map, Obstacles, Must-pass
% Points,Reference Trajectory ====
figure('Name', 'Complete RHP Visualization', 'Position', [200, 200, 1000, 800]);
hold on;
axis equal;
xlabel('X (meters)');
ylabel('Y (meters)');
title('Binary Occupancy Grid and Trajectories');
grid on;

% Re-built binaryOccupancyMap 
mapWidth = 25;    
mapHeight = 25;   
resolution = 10;  
local_map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Draw the stop line (black line)
for y = [6, 10, 14, 18]
    setOccupancy(local_map, [1:0.1:6; y*ones(1, length(1:0.1:6))]', 1);
end
setOccupancy(local_map, [1*ones(1, length(6:0.1:22)); 6:0.1:22]', 1);
setOccupancy(local_map, [1:0.1:6; 22*ones(1, length(1:0.1:6))]', 1);

for y = [6, 10, 14, 18]
    setOccupancy(local_map, [12:0.1:17; y*ones(1, length(12:0.1:17))]', 1);
end
setOccupancy(local_map, [17*ones(1, length(6:0.1:22)); 6:0.1:22]', 1);
setOccupancy(local_map, [12:0.1:17; 22*ones(1, length(12:0.1:17))]', 1);

% Draw the obstacle vehicle (black square)
markRectangle(local_map, 2, 7.1, 2.4, 1.8);
markRectangle(local_map, 2, 11.1, 2.4, 1.8);
markRectangle(local_map, 2, 19.1, 2.4, 1.8);
markRectangle(local_map, 13, 7.1, 2.4, 1.8);
markRectangle(local_map, 13, 19.1, 2.4, 1.8);
markRectangle(local_map, 8.1, 7.8, 1.8, 2.4);
markRectangle(local_map, 0, 2.1, 2.4, 1.8);

% Inflate map
inflate(local_map, 1.1);

% Show parking lot background
show(local_map);

% Draw obstacles (red circles) 
theta = linspace(0, 2*pi, 100);
r = 3; 
obstacles = [3.2 14.0; 3.2 20.0; 14.2 8.0; 14.2 20.0];
for i = 1:size(obstacles,1)
    plot(obstacles(i,1) + r*cos(theta), obstacles(i,2) + r*sin(theta), 'r-', 'LineWidth', 2);
end

% Must-pass waypoints (black crosses) 
wp = [0.2 20 20 8 8 13.2; 3 3 22 22 16 16; 0 pi/2 pi 3*pi/2 2*pi 2*pi];
plot(wp(1,:), wp(2,:), 'kx', 'MarkerSize', 10, 'LineWidth', 2);

% Reference Trajectory from Task 2.1 (red dashed line) 
[x_ref_full, y_ref_full, ~, ~, ~, ~, ~, ~, ~] = Task2_2_ref(0.1); 
plot(x_ref_full, y_ref_full, 'r--', 'LineWidth', 1.5);

% Optimized RHP trajectory (blue solid line) 
plot(x_all(1,:), x_all(2,:),'Color', [0.4 0.6 1],'LineWidth', 2);




% Local function to mark rectangles 
function markRectangle(map, x, y, w, h)
    xs = x : 0.05 : (x + w);
    ys = y : 0.05 : (y + h);
    [X, Y] = meshgrid(xs, ys);
    setOccupancy(map, [X(:), Y(:)], 1);
end


% Manually create a proper legend handles 
h1 = plot(NaN, NaN, 'r-', 'LineWidth', 2);      % Obstacles 
h2 = plot(NaN, NaN, 'kx', 'MarkerSize', 10, 'LineWidth', 2); % Must-pass waypoints
h3 = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);   % Reference trajectory 
h4 = plot(NaN, NaN, 'Color', [0.4 0.6 1],'LineWidth', 2);      % Optimized trajectory 

legend([h1 h2 h3 h4], {'Obstacles', 'Must-pass Waypoints', 'Reference Trajectory', 'Optimized Trajectory'}, 'Location', 'bestoutside');

