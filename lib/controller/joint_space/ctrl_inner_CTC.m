function tau = ctrl_inner_CTC(M_mat, C_vec, g_vec, theta_ddot_cmd)
% CTRL_INNER_CTC Calculates the physical joint torques required to achieve
% the commanded acceleration, effectively canceling system nonlinearities.
%
% Inputs:
%   M_mat          : nxn evaluated Mass/Inertia Matrix
%   C_vec          : nx1 evaluated Coriolis and Centrifugal vector
%   g_vec          : nx1 evaluated Gravity vector
%   theta_ddot_cmd : nx1 commanded joint acceleration from outer loop
%
% Outputs:
%   tau            : nx1 joint torques to be applied to the physical plant

    % 1. Computed Torque Control Law
    % The nonlinear terms (C_vec and g_vec) are added to cancel out the 
    % physical plant's natural dynamics. The M_mat scales the commanded 
    % acceleration to real physical torque (F = ma).
    tau = M_mat * theta_ddot_cmd + C_vec + g_vec;

end