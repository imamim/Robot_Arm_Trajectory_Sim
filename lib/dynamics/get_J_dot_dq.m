function J_dot_dq = get_J_dot_dq(q, dq, L1, L2)
    % GET_J_DOT_DQ Computes the Cartesian acceleration bias term (dJ*dq)
    % This represents the end-effector acceleration caused strictly by
    % joint velocities (Coriolis and centripetal effects in task space).
    %
    % Inputs:
    %   q:  2x1 vector of joint angles [q1; q2] in radians
    %   dq: 2x1 vector of joint velocities [dq1; dq2] in rad/s
    %   L1, L2: Link lengths
    %
    % Output:
    %   J_dot_dq: 2x1 Cartesian acceleration vector [x_bias; y_bias]
    
    q1 = q(1);
    q2 = q(2);
    dq1 = dq(1);
    dq2 = dq(2);
    
    % Pre-calculate trigonometric terms for computational speed
    cos_q1 = cos(q1);
    sin_q1 = sin(q1);
    cos_q12 = cos(q1 + q2);
    sin_q12 = sin(q1 + q2);
    
    % Pre-calculate squared velocities (centripetal terms)
    dq1_sq = dq1^2;
    dq12_sq = (dq1 + dq2)^2;
    
    % Analytical evaluation of dJ*dq
    J_dot_dq_x = -L1 * cos_q1 * dq1_sq - L2 * cos_q12 * dq12_sq;
    J_dot_dq_y = -L1 * sin_q1 * dq1_sq - L2 * sin_q12 * dq12_sq;
    
    J_dot_dq = [J_dot_dq_x; J_dot_dq_y];
end