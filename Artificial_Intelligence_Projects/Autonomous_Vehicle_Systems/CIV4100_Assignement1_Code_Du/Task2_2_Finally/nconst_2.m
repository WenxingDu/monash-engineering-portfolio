% nconst_2.m

function [ineqvalue, eqvalue] = nconst_2(eta, prob_info)
% ============================================================
% There are two main types of constraints:
% - Equality constraints:
%   Used to enforce system dynamics
%   All time steps require that the system state advances correctly according to 
%   the dynamics.
% - Inequality constraint:
%   Used for obstacle avoidance. Ensure that the vehicle will not enter the radius of the 
%   circular obstacle at any time
% ============================================================


% Extract the configuration parameters from prob_info
N  = prob_info.N;           % Optimization steps
Nx = prob_info.Nx;          % Number of state variables
Nu = prob_info.Nu           % Number of control variables
T  = prob_info.T;           % Total length of time domain
x0 = prob_info.x0;          % Initial state
delta = T / N;              % Single step duration

% Extract and reshape states and controls from eta 
x = reshape(eta(1:N*Nx), Nx, N);    
x = [x0, x];                         
u = reshape(eta((N+1)*Nx+1:end), Nu, N); 

% Equality constraints: System dynamics
% For each time step k, calculate the difference in system dynamics:
% x(k+1) = x(k) + delta * f(x(k), u(k))
eqvalue = zeros(N*Nx, 1);
for k = 1:N
    eqvalue((k-1)*Nx+1:k*Nx, 1) = x(:,k+1) - x(:,k) - delta * sys_h_2(x(:,k), u(:,k), prob_info);
end

% Inequality constraints: Obstacle avoidance
r = prob_info.radii;                 % Fixed obstacle radius
circles = prob_info.centres;         % Each center [cx cy]
num_circles = size(circles, 1);      % Number of obstacles
ineqvalue = zeros(N * num_circles, 1);  % Initialize inequality constraint values

% For each obstacle + each time step, construct a distance constraint:
% Ensure that the Euclidean distance between the vehicle and the center of the circle is ≥ r
for i = 1:num_circles
    cx = circles(i,1);
    cy = circles(i,2);
    for k = 1:N
        idx = (i-1)*N + k;
        
        ineqvalue(idx) = r^2 - (x(1,k+1) - cx)^2 - (x(2,k+1) - cy)^2;
    end
end

end
