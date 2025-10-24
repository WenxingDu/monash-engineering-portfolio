% cost_2.m

function value = cost_2(eta, prob_info)
% =============================================
% This cost function ensures that u(t) is as close as possible to:
% - Let the vehicle move along the reference trajectory (minimize x/y deviation)
% - Control as smooth as possible (avoid sharp)
% - The end point is aligned with the target angle accurately
% =======================================================================
% Returns the total cost corresponding to the given trajectory eta, 
% which includes two parts:
% - Running Cost: Tracking Error + Control Effort
% - Terminal Cost: Heading Angle Error


% Get information such as the number of discrete points, number of state variables, 
% number of control variables, total duration
N  = prob_info.N;
Nx = prob_info.Nx;
Nu = prob_info.Nu;
T  = prob_info.T;
dt = T / N;
% Error weighting factor
wp = prob_info.wp;      % Position tracking error weight
wh = prob_info.wh;      % Heading angle terminal error weight

% Unpack the front part of eta into the state matrix x, size is 5*(N+1)
% Unpack the latter part into the control matrix u, size is 2*N
x = reshape(eta(1:(N+1)*Nx), Nx, N+1);
u = reshape(eta((N+1)*Nx+1:end), Nu, N);

% Reference trajectory
x_ref = prob_info.x_ref;  % size is [3 * (N+1)] => [x_ref; y_ref; theta_ref]
r1 = x_ref(1,:);          % x-coordinate reference
r2 = x_ref(2,:);          % y-coordinate reference
r3 = x_ref(3,:);          % theta angle reference


% Running cost: tracking error + control effort
L = 0;
for k = 1:N
    % Control effort penalty 
    u_cost = 0.5 * (u(1,k)^2 + u(2,k)^2);

    % Position tracking error
    tracking_cost = wp * ((x(1,k) - r1(k))^2 + (x(2,k) - r2(k))^2);
    
    % Weighted Points
    L = L + (u_cost + tracking_cost) * dt;
end

% Terminal cost: heading error
heading_err = wrapToPi(r3(end) - x(3,end)); %The difference between the expected angle and 
                                            % the actual angle at the end
H = wh * heading_err^2;      % Weighted square penalty

% Total cost
value = L + H;
end
