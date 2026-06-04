function [tau_applied, err, tau_cmd] = master_controller_OSC(t, x, traj_state, params)
% MASTER_CONTROLLER_OSC Evaluates the Operational Space Control law.

    % 1. Unpack the augmented state
    theta = x(1:2);
    theta_dot = x(3:4);
    err_int = x(5:6);

    % 2. Forward Kinematics for Cartesian Position and Velocity
    L1 = params.L(1);
    L2 = params.L(2);
    
    % End-effector position
    x1 = L1 * cos(theta(1));
    y1 = L1 * sin(theta(1));
    x2 = x1 + L2 * cos(theta(1) + theta(2));
    y2 = y1 + L2 * sin(theta(1) + theta(2));
    x_cart = [x2; y2];
    
    % Jacobian and Cartesian velocity
    J = get_jacobian(theta, L1, L2);
    x_dot_cart = J * theta_dot;

    % 3. Unpack the injected Cartesian trajectory
    x_d        = traj_state(1:2);
    x_d_dot    = traj_state(3:4);
    x_d_ddot   = traj_state(5:6);

    % 4. Call Outer Loop (PID) in Task Space
    [x_ddot_cmd, err, ~] = ctrl_outer_PID_OSC(x_d, x_d_dot, x_d_ddot, x_cart, x_dot_cart, err_int, params);

    % 5. Evaluate Dynamic Matrices and J_dot_dq
    M = compute_M(theta, params);
    c = compute_c(theta, theta_dot, params);
    g = compute_g(theta, params);
    J_dot_dq = get_J_dot_dq(theta, theta_dot, L1, L2);

    % 6. Call Inner Loop (OSC) to get commanded torque
    tau_cmd = ctrl_inner_OSC(M, c, g, J, J_dot_dq, x_ddot_cmd);
    
    % 7. Apply Actuator Saturation
    tau_applied = max(min(tau_cmd, params.tau_max), -params.tau_max);

end