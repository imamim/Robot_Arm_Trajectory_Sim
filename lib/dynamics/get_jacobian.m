function J = get_jacobian(q, L1, L2)
    % GET_JACOBIAN Computes the 2x2 Space Jacobian for a 2-DOF planar arm
    % Inputs:
    %   q:  2x1 vector of joint angles [q1; q2] in radians
    %   L1, L2: Link lengths
    % Output:
    %   J:  2x2 Analytical Space Jacobian
    
    q1 = q(1);
    q2 = q(2);
    
    % Analytical derivatives of the forward kinematics
    J11 = -L1*sin(q1) - L2*sin(q1+q2);
    J12 = -L2*sin(q1+q2);
    J21 = L1*cos(q1) + L2*cos(q1+q2);
    J22 = L2*cos(q1+q2);
    
    J = [J11, J12; 
         J21, J22];
end