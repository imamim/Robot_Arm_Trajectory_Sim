function J_inv_dls = get_damped_inverse(J, lambda_max, epsilon)
    % GET_DAMPED_INVERSE Computes the DLS inverse of the Jacobian
    % Inputs:
    %   J:          2x2 Jacobian matrix
    %   lambda_max: Maximum damping factor (e.g., 0.1 to 0.5)
    %   epsilon:    Manipulability threshold to activate damping (e.g., 0.05)
    % Output:
    %   J_inv_dls:  Singularity-robust inverse Jacobian
    
    % Calculate Yoshikawa's Manipulability Measure
    w = sqrt(max(0, det(J * J'))); % max(0, ...) prevents complex numbers due to float errors
    
    % Dynamically calculate damping factor
    if w < epsilon
        % Nearing singularity: Apply smooth damping
        lambda = lambda_max * (1 - (w/epsilon)^2);
    else
        % Far from singularity: Pure inverse (lambda = 0)
        lambda = 0;
    end
    
    % Damped Least Squares Equation
    I = eye(size(J, 1));
    % MATLAB optimal matrix inversion (equivalent to J' * inv(J*J' + lambda^2*I))
    J_inv_dls = J' / (J * J' + (lambda^2) * I); 
end