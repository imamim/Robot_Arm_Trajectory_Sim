function options = get_sim_options(tspan, x0, trajectory_func, solverTol)
% GET_SIM_OPTIONS Packages simulation settings into a standardized struct.
%
% Inputs:
%   tspan           - 1x2 vector [t_start, t_end] defining simulation time.
%   x0              - 6x1 initial augmented state vector.
%   trajectory_func - Function handle for the reference trajectory @(t).
%   solverTol       - (Optional) ODE solver tolerance. Defaults to 1e-5.
%
% Outputs:
%   options         - Struct containing the parsed simulation settings.

    % Default tolerance if not explicitly provided
    if nargin < 4
        solverTol = 1e-5;
    end

    options = struct();
    options.tspan = tspan;
    options.x0 = x0;
    options.trajectory = trajectory_func;
    options.solverTol = solverTol;

end