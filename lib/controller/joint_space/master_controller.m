function [tau_applied, err, tau_cmd] = master_controller(t, x, traj_state, params)
% MASTER_CONTROLLER Evaluates the control law. Trajectory is injected from outside.

    % 1. Unpack the augmented state
    theta = x(1:2);
    theta_dot = x(3:4);
    err_int = x(5:6);

    % 2. Unpack the injected trajectory
    theta_d      = traj_state(1:2);
    theta_d_dot  = traj_state(3:4);
    theta_d_ddot = traj_state(5:6);

    % 3. Call Outer Loop (PID) 
    [theta_ddot_cmd, err, ~] = ctrl_outer_PID(theta_d, theta_d_dot, theta_d_ddot, theta, theta_dot, err_int, params);

    % 4. Evaluate Dynamic Matrices
    M = compute_M(theta, params);
    c = compute_c(theta, theta_dot, params);
    g = compute_g(theta, params);

    % 5. Call Inner Loop (CTC) to get commanded torque
    tau_cmd = ctrl_inner_CTC(M, c, g, theta_ddot_cmd);
    
    % 6. Apply Actuator Saturation
    tau_applied = max(min(tau_cmd, params.tau_max), -params.tau_max);

end