
function [x_ref, y_ref, theta_ref, s_ref, v_ref, a_ref, omega_ref, alpha_ref, t_ref] = Task2_1_ref(dt)
% =========================================================================
% Task 2.1 - Smoothed End Version
% =========================================================================

if nargin < 1
    dt = 0.1;
end

% === Loading maps and must-pass points ===
load('task1_1_map.mat');
inflatedMap = copy(map);
inflate(inflatedMap, 1.1);

% === Modified must-pass points with helper points at the end ===
wp_base = [0.2  5   20  20  15   8   8   13.2;
             3   1    3  22  24  22  16   16];

% === Interpolation auxiliary points (smoothing with arcs) ===
arc_resolution = 10; angle_thresh = pi/2;
wp_aug = wp_base(:,1);
for i = 2:size(wp_base,2)-1
    p0 = wp_base(:,i-1);
    p1 = wp_base(:,i);
    p2 = wp_base(:,i+1);
    v1 = p1 - p0; v2 = p2 - p1;
    theta1 = atan2(v1(2), v1(1));
    theta2 = atan2(v2(2), v2(1));
    delta_theta = wrapToPi(theta2 - theta1);
    if abs(delta_theta) > angle_thresh
        arc_pts = arc_through_three(p0, p1, p2, arc_resolution);
        wp_aug = [wp_aug, arc_pts(:,2:end-1), p2];
    else
        wp_aug = [wp_aug, p1];
    end
end
if ~isequal(wp_aug(:,end), wp_base(:,end))
    wp_aug = [wp_aug, wp_base(:,end)];
end

% === Occupancy check ===
valid = ~checkOccupancy(inflatedMap, wp_aug');
wp = wp_aug(:,valid);

% ===  s → x(s), y(s) ===
dist = sqrt(sum(diff(wp,1,2).^2));
s_wp = [0, cumsum(dist)];
cs_x = spline(s_wp, wp(1,:));
cs_y = spline(s_wp, wp(2,:));

% === Use local three-stage speed control ===
vmax = 3; amax = 2; dmax = 3;
s_all = []; v_all = []; a_all = []; t_all = [];
s_offset = 0; t_offset = 0;
for i = 1:length(dist)
    s_i = dist(i);
    [t_seg, s_seg, v_seg, a_seg] = trapezoidal_profile(s_i, 0, 0, vmax, amax, -dmax, dt);
    s_all = [s_all, s_seg + s_offset];
    v_all = [v_all, v_seg];
    a_all = [a_all, a_seg];
    t_all = [t_all, t_seg + t_offset];
    s_offset = s_offset + s_i;
    t_offset = t_offset + t_seg(end);
end

% === interpolated trajectory ===
x_ref = ppval(cs_x, s_all);
y_ref = ppval(cs_y, s_all);

% === filter obstacles ===
valid2 = ~checkOccupancy(inflatedMap, [x_ref(:), y_ref(:)]);
x_ref = x_ref(valid2);
y_ref = y_ref(valid2);
s_ref = s_all(valid2);
t_ref = t_all(valid2);
v_ref = v_all(valid2);
a_ref = a_all(valid2);

% === headings ===
dx = ppval(fnder(cs_x), s_ref);
dy = ppval(fnder(cs_y), s_ref);
theta_ref = atan2(dy, dx);
omega_ref = [0 diff(theta_ref)/dt];
omega_ref = sgolayfilt(omega_ref, 3, 11);
alpha_ref = [0 diff(omega_ref)/dt];

end


function arc_pts = arc_through_three(p1, p2, p3, N)
A = 2 * [p2'-p1'; p3'-p1'];
b = [norm(p2-p1)^2; norm(p3-p1)^2];
c = (A\b)' + p1';
r = norm(c' - p1);
theta1 = atan2(p1(2)-c(2), p1(1)-c(1));
theta3 = atan2(p3(2)-c(2), p3(1)-c(1));
dtheta = wrapToPi(theta3 - theta1);
if dtheta < 0, dtheta = dtheta + 2*pi; end
theta = linspace(theta1, theta1 + dtheta, N);
x_arc = c(1) + r * cos(theta);
y_arc = c(2) + r * sin(theta);
arc_pts = [x_arc; y_arc];
end


function [t_grid, s, v, a] = trapezoidal_profile(S, v0, v1, v_max, a_pos, a_neg, dt)
ta = (v_max - v0)/a_pos;
td = (v_max - v1)/abs(a_neg);
sa = v0*ta + 0.5*a_pos*ta^2;
sd = v_max*td + 0.5*a_neg*td^2;
sc = S - sa - sd;

if sc < 0
    A = 1/a_pos + 1/abs(a_neg);
    B = -v0/a_pos - v1/abs(a_neg);
    C = (v0^2)/(2*a_pos) + (v1^2)/(2*abs(a_neg)) - S;
    v_peak = (-B + sqrt(B^2 - 4*A*C))/(2*A);
    ta = (v_peak - v0)/a_pos;
    td = (v_peak - v1)/abs(a_neg);
    tc = 0; v_max = v_peak;
else
    tc = sc / v_max;
end
T = ta + tc + td;
t_grid = 0:dt:T;
a = zeros(size(t_grid));
a(t_grid<=ta) = a_pos;
a(t_grid>ta & t_grid<=ta+tc) = 0;
a(t_grid>ta+tc) = a_neg;
v = v0 + cumtrapz(t_grid, a);
s = v0*t_grid + cumtrapz(t_grid, v);
end
