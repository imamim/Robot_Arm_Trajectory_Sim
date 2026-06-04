function [Lambda, mu, p] = get_op_space_dynamics(M, c_vec, g_vec, J, J_dot_dq)
    % GET_OP_SPACE_DYNAMICS Computes Operational Space dynamic matrices
    %
    % Inputs:
    %   M:        2x2 Joint Space Inertia Matrix
    %   c_vec:    2x1 Joint Space Coriolis/Centrifugal vector (C*dq)
    %   g_vec:    2x1 Joint Space Gravity vector
    %   J:        2x2 Analytical Space Jacobian
    %   J_dot_dq: 2x1 Cartesian acceleration vector from joint velocities (dJ*dq)
    %

    % Outputs:
    %   Lambda:   2x2 Operational Space Inertia Matrix
    %   mu:       2x1 Operational Space Coriolis/Centrifugal vector
    %   p:        2x1 Operational Space Gravity vector

    % 1. Inverse of the Joint Space Inertia Matrix
    M_inv = inv(M); 
    
    % 2. Calculate Lambda
    % Using safe_jacobian_inverse to apply DLS and prevent torque explosion near singularities
    Lambda_inv = J * M_inv * J';
    Lambda = safe_jacobian_inverse(Lambda_inv, 0.1, 0.05);
    
    % 3. Calculate Dynamically Consistent Inverse Transpose
    % This maps joint-space forces to task-space without blowing up like inv(J)' does.
    J_bar_T = Lambda * J * M_inv; 
    
    % 4. Calculate mu vector
    mu = J_bar_T * c_vec - Lambda * J_dot_dq;
    
    % 5. Calculate p (gravity) vector
    p = J_bar_T * g_vec;
end