function tau = ctrl_inner_OSC(M_mat, C_vec, g_vec, J, J_dot_dq, x_ddot_cmd)
    % CTRL_INNER_OSC Calculates the joint torques required to achieve
    % the commanded Cartesian acceleration, effectively canceling operational
    % space nonlinearities.
    %
    % Inputs:
    %   M_mat      : nxn Joint Space Mass/Inertia Matrix
    %   C_vec      : nx1 Joint Space Coriolis and Centrifugal vector
    %   g_vec      : nx1 Joint Space Gravity vector
    %   J          : mxn Analytical Space Jacobian
    %   J_dot_dq   : mx1 Cartesian acceleration vector from joint velocities (dJ*dq)
    %   x_ddot_cmd : mx1 Commanded Cartesian acceleration (x_ddot_d + Kd*e_dot + Kp*e)
    %
    % Outputs:
    %   tau        : nx1 Joint torques to be applied to the physical plant motors

    % 1. Map Joint Space Dynamics to Operational Space Dynamics
    % We call our robust utility function to project M, C, and g into the task space.
    [Lambda, mu, p] = get_op_space_dynamics(M_mat, C_vec, g_vec, J, J_dot_dq);
    
    % 2. Computed Force Law (Operational Space)
    % Calculates the generalized force F required at the end-effector.
    % Lambda scales the commanded acceleration to real physical force (F = ma).
    % mu and p cancel out the Cartesian nonlinearities and gravity.
    F = Lambda * x_ddot_cmd + mu + p;
    
    % 3. Map Cartesian Force to Joint Torques
    % Use the Jacobian transpose to distribute the required end-effector 
    % force into the respective joint motors.
    tau = J' * F;
    
end