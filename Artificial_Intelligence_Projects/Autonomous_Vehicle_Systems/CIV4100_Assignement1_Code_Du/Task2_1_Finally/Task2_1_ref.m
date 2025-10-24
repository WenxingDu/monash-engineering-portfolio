
% Task2.1 - Reference trajectory generation 


function [x_ref, y_ref, theta_ref, s_ref, v_ref, a_ref, omega_ref, alpha_ref, t_ref] = Task2_1_ref(dt)
% This function generates a smooth, kinematically feasible reference trajectory 
% based on a set of must-pass waypoints. It includes:
% - x_ref, y_ref: interpolated global positions along the trajectory
% - theta_ref: heading direction
% - v_ref, a_ref: velocity and acceleration profile
% - omega_ref, alpha_ref: angular velocity and acceleration
% The trajectory avoids sharp turns by inserting arc segments where needed
% It also applies a three-stage velocity profile (acceleration – cruise – deceleration) to ensure physical feasibility
% This trajectory will later be used in Task2.2 as well


dt = 0.1;


% === Load the map and inflate it to account for vehicle size  ===
load('task1_1_map.mat');
inflatedMap = copy(map);
inflate(inflatedMap, 1.1);

% === Set the waypoints, including auxiliary points for steering control ===
wp_base = [0.2  5   20  20  15   8   8   13.2;
             3   1    3  22  24  22  16   16];
% Each column represents an (x, y) coordinate

% === Smoothing with arcs ===
% Use arc interpolation to smooth the path and avoid situations such as 
% 90-degree blind spots that do not conform to vehicle kinematics
arc_resolution = 10;        % Insert 10 interpolation points into each arc
angle_thresh = pi/2;        % If the angle is greater than 90 degrees, perform arc smoothing
wp_aug = wp_base(:,1);      % Initialize the smoothed path, starting from the first point

for i = 2:size(wp_base,2)-1    % Traverse each group of 3 points in the path to determine whether an arc segment needs to be inserted
    p0 = wp_base(:,i-1);           % Previous point
    p1 = wp_base(:,i);             % Current point
    p2 = wp_base(:,i+1);           % Next point
    v1 = p1 - p0;                  % Vector: from p0 to p1   
    v2 = p2 - p1;                  % Vector: from p1 to p2    
    theta1 = atan2(v1(2), v1(1));  % Angle of the first vector
    theta2 = atan2(v2(2), v2(1));  % Angle of the second vector
    delta_theta = wrapToPi(theta2 - theta1);         % Angle, converted to [-pi, pi]
    if abs(delta_theta) > angle_thresh               % If it is a sharp turn, insert an arc segment between these three points
        arc_pts = arc_through_three(p0, p1, p2, arc_resolution);   
        wp_aug = [wp_aug, arc_pts(:,2:end-1), p2];
        % The inserted points exclude the first and last ones to avoid duplication
    else
        % Otherwise directly connect to the current point
        wp_aug = [wp_aug, p1];
    end
end

% Ensure that the end target point is added
if ~isequal(wp_aug(:,end), wp_base(:,end))
    wp_aug = [wp_aug, wp_base(:,end)];
end


% === Occupancy check ===
valid = ~checkOccupancy(inflatedMap, wp_aug');
wp = wp_aug(:,valid);

% ===  s → x(s), y(s) ===
dist = sqrt(sum(diff(wp,1,2).^2));     % Calculate the length of each path
s_wp = [0, cumsum(dist)];              % Cumulative path length,
cs_x = spline(s_wp, wp(1,:));          % Interpolate x(s) using cubic spline
cs_y = spline(s_wp, wp(2,:));          % Interpolate y(s) using cubic spline

% === Use local three-stage speed control ===

% Set the maximum speed, acceleration, and deceleration
vmax = 3; amax = 2; dmax = 3;
s_all = []; v_all = []; a_all = []; t_all = [];    % Initialize all track quantities
s_offset = 0; t_offset = 0;                        % Cumulative offset, used to splice each segment
for i = 1:length(dist)              
    s_i = dist(i);          % The length of the current segment
    % Use the trapezoidal profile function to calculate the time, s(t), v(t), and a(t) of this trajectory
    [t_seg, s_seg, v_seg, a_seg] = trapezoidal_profile(s_i, 0, 0, vmax, amax, -dmax, dt);
    s_all = [s_all, s_seg + s_offset];     % Global s(t), velocity, acceleration, deceleration, time,
    v_all = [v_all, v_seg];
    a_all = [a_all, a_seg];
    t_all = [t_all, t_seg + t_offset];
    s_offset = s_offset + s_i;             % Update path length offset
    t_offset = t_offset + t_seg(end);      % Update time offset
end

% === Interpolated trajectory ===
% Insert the cumulative path length s_all into the previously constructed x(s) and y(s) curves 
% to obtain the coordinate position corresponding to each time step on the trajectory
x_ref = ppval(cs_x, s_all);
y_ref = ppval(cs_y, s_all);

% === Filter obstacles ===
% Prevent the trajectory from crossing obstacles and remove every unreasonable point
valid2 = ~checkOccupancy(inflatedMap, [x_ref(:), y_ref(:)]);
x_ref = x_ref(valid2);
y_ref = y_ref(valid2);
s_ref = s_all(valid2);
t_ref = t_all(valid2);
v_ref = v_all(valid2);
a_ref = a_all(valid2);

% === Headings ===
% Calculate heading angle (theta), angular velocity (omega), angular acceleration (alpha)
% The heading angle theta represents the direction the vehicle is facing at this moment
% Angular velocity omega indicates the speed of turning; angular acceleration alpha indicates the trend of turning

dx = ppval(fnder(cs_x), s_ref);
dy = ppval(fnder(cs_y), s_ref);
theta_ref = atan2(dy, dx);
omega_ref = [0 diff(theta_ref)/dt];        % Calculate angular velocity (difference approximation)
omega_ref = sgolayfilt(omega_ref, 3, 11);  % Use Savitzky-Golay filter to smooth the angular velocity
alpha_ref = [0 diff(omega_ref)/dt];

end


function arc_pts = arc_through_three(p1, p2, p3, N)
% Input: p1, p2, p3 are the coordinates of three points, and the arc is required to pass through these three points
% N represents the number of interpolation points generated
% Output: arc_pts is a 2×N array, each column is a point on the arc after interpolation

% The method of geometric three-point circle determination to solve the center c
A = 2 * [p2'-p1'; p3'-p1'];
b = [norm(p2-p1)^2; norm(p3-p1)^2];
c = (A\b)' + p1';               % Find the center position c
r = norm(c' - p1);              % Radius r, which is the distance from the center of the circle to any point
theta1 = atan2(p1(2)-c(2), p1(1)-c(1));     % p1 polar angle relative to the center of the circle
theta3 = atan2(p3(2)-c(2), p3(1)-c(1));     % p3 Polar angle relative to the center of the circle
dtheta = wrapToPi(theta3 - theta1);         % Angle Difference, use wrapToPi to wrap it to the range [-pi, pi]

if dtheta < 0, dtheta = dtheta + 2*pi; end      % If it is a negative angle, it becomes positive (clockwise)
theta = linspace(theta1, theta1 + dtheta, N);   % Generate N points starting from theta1 with uniform angle steps
x_arc = c(1) + r * cos(theta);       % Polar coordinates converted to Cartesian coordinates
y_arc = c(2) + r * sin(theta);
arc_pts = [x_arc; y_arc];            % Returns the arc point set
end



function [t_grid, s, v, a] = trapezoidal_profile(S, v0, v1, v_max, a_pos, a_neg, dt)
% Input: path length S, starting speed v0, ending speed v1
% Maximum speed v_max, acceleration a_pos, deceleration a_neg
% Time step dt
% Output: t_grid: time series; s(t): position; v(t): speed; a(t): acceleration

ta = (v_max - v0)/a_pos;           % Acceleration phase time
td = (v_max - v1)/abs(a_neg);      % Deceleration phase time
sa = v0*ta + 0.5*a_pos*ta^2;       % Distance traveled during the acceleration phase
sd = v_max*td + 0.5*a_neg*td^2;    % Distance traveled during deceleration phase
sc = S - sa - sd;                  % % The remaining distance is used for the constant speed stage

if sc < 0
% If the distance is too short to reach the maximum speed, use the triangular speed curve that "removes the constant speed stage"

    A = 1/a_pos + 1/abs(a_neg);           % Solve for the most reasonable intermediate speed v_peak
    B = -v0/a_pos - v1/abs(a_neg);
    C = (v0^2)/(2*a_pos) + (v1^2)/(2*abs(a_neg)) - S;
    v_peak = (-B + sqrt(B^2 - 4*A*C))/(2*A);
    ta = (v_peak - v0)/a_pos;
    td = (v_peak - v1)/abs(a_neg);
    tc = 0; v_max = v_peak;               % No constant speed segment
else
    tc = sc / v_max;            % Constant speed phase time
end
T = ta + tc + td;                 % Total Time
t_grid = 0:dt:T;                  % Timeline
a = zeros(size(t_grid));          % Initialize acceleration
a(t_grid<=ta) = a_pos;            % Acceleration segment
a(t_grid>ta & t_grid<=ta+tc) = 0; % Constant speed segment
a(t_grid>ta+tc) = a_neg;          % Deceleration section
v = v0 + cumtrapz(t_grid, a);     % Integrate to get the velocity v(t) 
s = v0*t_grid + cumtrapz(t_grid, v);  % Integrate again to get the position s(t)
end 










