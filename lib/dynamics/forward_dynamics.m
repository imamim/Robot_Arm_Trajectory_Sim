function dx = forward_dynamics(t, x, tau, p)
% FORWARD_DYNAMICS Computes the state derivatives for the 2-DoF robot.
%
% Inputs:
%   t   - Current time [s] (required signature for ODE solvers)
%   x   - 4x1 state vector [theta1; theta2; theta1_dot; theta2_dot]
%   tau - 2x1 joint torque vector [Nm]
%   p   - Parameter structure from get_robot_params()
% Outputs:
%   dx  - 4x1 state derivative vector [theta_dot; theta_ddot]

    % 1. Unpack the state vector
    theta = x(1:2);
    theta_dot = x(3:4);

    % 2. Compute dynamic matrices (Calling Milestone 2 functions)
    M = compute_M(theta, p);
    c = compute_c(theta, theta_dot, p);
    g = compute_g(theta, p);

    % 3. Solve Forward Dynamics for joint accelerations
    % Mathematically: theta_ddot = inv(M) * (tau - c - g)
    % Architecturally: Use the backslash operator (\) for numerical stability
    theta_ddot = M \ (tau - c - g);

    % 4. Pack the derivative vector for the integrator
    dx = [theta_dot; 
          theta_ddot];
end