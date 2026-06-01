function c = compute_c(theta, theta_dot, p)
% COMPUTE_C Calculates the 2x1 Coriolis and Centrifugal force vector.
%
% Inputs:
%   theta     - 2x1 vector of joint angles [rad]
%   theta_dot - 2x1 vector of joint velocities [rad/s]
%   p         - Parameter structure from get_robot_params()
% Outputs:
%   c         - 2x1 Velocity-product vector [Nm]

    % Extract parameters
    m2 = p.m(2); 
    L1 = p.L(1); L2 = p.L(2);
    
    theta2 = theta(2);
    
    th1_d = theta_dot(1); 
    th2_d = theta_dot(2);

    % Vector Elements (Derived from the uploaded slide)
    c1 = -m2*L1*L2*sin(theta2) * (2*th1_d*th2_d + th2_d^2);
    c2 = m2*L1*L2*th1_d^2 * sin(theta2);

    % Construct Vector
    c = [c1; 
         c2];
end