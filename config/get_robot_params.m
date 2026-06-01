function p = get_robot_params()
% GET_ROBOT_PARAMS Initializes the physical parameters of the 2-DoF robot.
%
% Outputs:
%   p - Structured array with vectorized kinematic and dynamic parameters.

    p = struct();

    % Environment
    p.g = 9.81;         % Gravitational acceleration (m/s^2)

    % Kinematics: Link Lengths (m)
    p.L = [0.6; 
           0.4]; 

    % Dynamics: Link Point Masses (kg)
    p.m = [0.5; 
           0.2]; 
           
end