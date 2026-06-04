function tau = controller_pd_gravity(t, x, p, q_target, Kp, Kd)
% CONTROLLER_PD_GRAVITY Computes control torque to reach a target configuration.
%
% Inputs:
%   t        - Current time [s]
%   x        - Current state [theta1; theta2; theta1_dot; theta2_dot]
%   p        - Parameter struct from get_robot_params()
%   q_target - 2x1 Desired joint angles [rad]
%   Kp       - 2x2 Proportional gain matrix
%   Kd       - 2x2 Derivative gain matrix
% Outputs:
%   tau      - 2x1 Commanded joint torque vector [Nm]

    % 1. Extract current states
    q = x(1:2);
    q_dot = x(3:4);

    % 2. Calculate errors
    % We assume the target is a static setpoint, so desired velocity is 0
    e = q_target - q;
    e_dot = [0; 0] - q_dot;

    % 3. Calculate Gravity Compensation
    % Reusing your Milestone 2 library
    g_comp = compute_g(q, p);

    % 4. Compute Control Law
    tau = Kp * e + Kd * e_dot + g_comp;
    
    % --- Optional Real-World Safeguard: Torque Saturation ---
    % Real motors have torque limits. You can uncomment these lines 
    % to prevent mathematically infinite torques from crashing the simulation.
    % max_tau = 50; % Nm limit
    % tau = max(min(tau, max_tau), -max_tau);
end