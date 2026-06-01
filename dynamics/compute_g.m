function g_vec = compute_g(theta, p)
% COMPUTE_G Calculates the 2x1 Gravity torque vector.
%
% Inputs:
%   theta - 2x1 vector of joint angles [rad]
%   p     - Parameter structure from get_robot_params()
% Outputs:
%   g_vec - 2x1 Gravity vector [Nm]

    % Extract parameters
    g = p.g;
    m1 = p.m(1); m2 = p.m(2);
    L1 = p.L(1); L2 = p.L(2);
    
    theta1 = theta(1);
    theta2 = theta(2);

    % Vector Elements (Derived from the uploaded slide)
    g1 = (m1 + m2)*L1*g*cos(theta1) + m2*g*L2*cos(theta1 + theta2);
    g2 = m2*g*L2*cos(theta1 + theta2);

    % Construct Vector
    g_vec = [g1; 
             g2];
end