% Quick script to test animation
addpath('config', 'dynamics', 'core', 'vis');

params = get_robot_params();

% Re-use the free fall test configuration
options.tspan = [0 100];
options.x0 = [pi/2; 0; 0; 0]; % Start pointing straight up, at rest
options.solverTol = 1e-5;

% Run simulation engine
res = simulateRobot(params, @(t,x,p) [0; 0], options);

% Run the new animation module
animate_robot(res, params);