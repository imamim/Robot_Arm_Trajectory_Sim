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

    params = get_robot_params();

    % --- NEW: Controller Parameters ---
   p.Kp = 100 * eye(2);    % Proportional Gain
   p.Kd = 20 * eye(2);     % Derivative Gain
   p.Ki = 50 * eye(2);     % Integral Gain
   p.int_limit = 10;       % Anti-windup limit

           
end