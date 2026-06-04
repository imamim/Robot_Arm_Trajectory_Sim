function M = compute_M(theta, p)
% COMPUTE_M Calculates the 2x2 symmetric Mass/Inertia Matrix.
%
% Inputs:
%   theta - 2x1 vector of joint angles [rad]
%   p     - Parameter structure from get_robot_params()
% Outputs:
%   M     - 2x2 Mass matrix [kg*m^2]

    % Extract parameters for readability
    m1 = p.m(1); L1 = p.L(1);
    m2 = p.m(2); L2 = p.L(2);
    
    theta2 = theta(2);

    % Matrix Elements (Derived from the uploaded slide)
    M11 = m1*L1^2 + m2*(L1^2 + 2*L1*L2*cos(theta2) + L2^2);
    M12 = m2*(L1*L2*cos(theta2) + L2^2);
    M21 = M12; % The mass matrix is always symmetric
    M22 = m2*L2^2;

    % Construct Matrix
    M = [M11, M12;
         M21, M22];
end