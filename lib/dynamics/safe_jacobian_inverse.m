function J_inv_safe = safe_jacobian_inverse(J, epsilon_max, threshold)
    % SAFE_JACOBIAN_INVERSE Computes a singularity-robust pseudo-inverse of a Jacobian.
    % Uses Damped Least Squares (Levenberg-Marquardt) via SVD.
    %
    % Inputs:
    %   J           - Analytical or geometric Jacobian matrix (m x n)
    %   epsilon_max - Maximum damping factor (e.g., 0.1)
    %   threshold   - Singular value threshold to trigger damping (e.g., 0.05)
    %
    % Output:
    %   J_inv_safe  - Safely inverted Jacobian (n x m)

    % Compute Singular Value Decomposition (SVD)
    [U, S, V] = svd(J);
    s = diag(S);
    
    % Find minimum singular value
    s_min = min(s);
    
    % Calculate dynamic damping factor (activates only near singularity)
    if s_min >= threshold
        epsilon = 0; % Far from singularity: use pure pseudo-inverse
    else
        % Smoothly increase damping as s_min approaches 0
        epsilon = epsilon_max * sqrt(1 - (s_min / threshold)^2);
    end
    
    % Compute damped singular values: s / (s^2 + epsilon^2)
    s_damped = s ./ (s.^2 + epsilon.^2);
    
    % Reconstruct the damped pseudo-inverse matrix
    S_inv = zeros(size(J, 2), size(J, 1));
    S_inv(1:length(s), 1:length(s)) = diag(s_damped);
    
    J_inv_safe = V * S_inv * U';
end