% sys_h_2.m

function dxdt = sys_h_2(x, u, prob_info)
% SYS_H: System dynamics function for vehicle motion.
% Inputs:
%   x:current state [x; y; theta; v; cost]
%   u:current control input [delta; a]
%   prob_info:struct of problem settings 
% Output:
%   dxdt：time derivative of the state

    L = 2; % length from rear wheel axle to the front axle

    dxdt = [x(4,1)*cos(x(3,1)); ...     % x_dot, the speed of the vehicle in the x direction
            x(4,1)*sin(x(3,1)); ...     % y_dot, the speed of the vehicle in the y direction
            (x(4,1)/L)*tan(u(1,1)); ... % theta_dot, the rate of change of the heading angle, 
                                        % which indicates how the vehicle is turning
            u(2,1); ...                 % v_dot, acceleration
            0.5*u(1,1)^2 + 0.5*u(2,1)^2]; % cost accumulation, this represents "control effort": the larger the 
                                          % angle and acceleration, the higher the cost (0.5 is the weight).
end

