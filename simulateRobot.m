function results = simulateRobot(params, controller_func, options)
% SIMULATEROBOT Executes a closed-loop simulation of the 2-DoF robot.
%
% Inputs:
%   params          - Struct containing robot physical parameters
%   controller_func - Function handle: tau = f(t, x, params)
%   options         - Struct with simulation settings (tspan, x0, etc.)
%
% Outputs:
%   results         - Struct containing time (t), states (x), and torques (tau)

    % 1. Extract simulation options (with defaults for safety)
    if isfield(options, 'tspan'), tspan = options.tspan; else, tspan = [0 5]; end
    if isfield(options, 'x0'), x0 = options.x0; else, x0 = zeros(6,1); end
    if isfield(options, 'solverTol'), tol = options.solverTol; else, tol = 1e-5; end

    % 2. Setup ODE Solver Options
    ode_opts = odeset('RelTol', tol, 'AbsTol', tol);

    % 3. Define the internal Plant Wrapper for ode45
    % ode45 requires the exact signature @(t, x). We wrap the controller 
    % and the forward dynamics together here.
    function dx = internal_plant(t, x)
        % Compute control torque
        tau = controller_func(t, x, params);
        % Compute state derivatives
        dx = forward_dynamics(t, x, tau, params);
    end

    % 4. Execute Numerical Integration
    disp('Starting simulation...');
    tic;
    [t_out, x_out] = ode45(@internal_plant, tspan, x0, ode_opts);
    sim_time = toc;
    fprintf('Simulation completed in %.3f seconds.\n', sim_time);

    % 5. Post-Process: Reconstruct Control Torques
    % ode45 does not return the internal values of tau. We must re-evaluate 
    % the controller over the resulting state trajectory to log the applied torques.
    N = length(t_out);
    tau_out = zeros(N, 2);
    for i = 1:N
        % Note: x_out is returned as row vectors by ode45, so we transpose it
        tau_out(i, :) = controller_func(t_out(i), x_out(i, :)', params)';
    end

    % 6. Pack Results
    results = struct();
    results.t = t_out;
    results.x = x_out;         % [N x 4] matrix
    results.tau = tau_out;     % [N x 2] matrix
    results.params = params;   % Pass params through for the animation module
end