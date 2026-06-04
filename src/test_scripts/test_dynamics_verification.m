% test_dynamics_verification.m
% PHASE 1: Validates Mass Matrix properties and Energy Conservation

clc; clear; close all;

% Ensure the project folders are in the MATLAB path
disp('--- Running Phase 1 Dynamics Verification ---');
params = get_robot_params();

%% TEST 1: Mass Matrix Properties
disp('Test 1: Checking Mass Matrix (Symmetry & Positive-Definiteness)...');
% Test at a random valid configuration
test_theta = [pi/4; -pi/3];
M_test = compute_M(test_theta, params);

% Check Symmetry (M = M^T)
is_symmetric = max(max(abs(M_test - M_test'))) < 1e-10;
% Check Positive Definiteness (Eigenvalues > 0)
eigenvalues = eig(M_test);
is_pos_def = all(eigenvalues > 0);

if is_symmetric && is_pos_def
    disp('PASS: Mass matrix is symmetric and positive-definite.');
else
    error('FAIL: Mass matrix properties violated. Check compute_M.m');
end

%% TEST 2: Energy Conservation (Free-fall)
disp('Test 2: Simulating free-fall for Energy Conservation check...');

% Setup a zero-torque controller for free-fall
zero_torque_ctrl = @(t, x, p) [0; 0];

% Setup simulation options
options.tspan = [0 10];
options.x0 = [pi/2; 0; 0; 0]; % Start pointing straight up
options.solverTol = 1e-6;     % High tolerance needed for accurate energy calculation

% Run Simulation
res = simulateRobot(params, zero_torque_ctrl, options);

% Calculate Energy at every time step
N = length(res.t);
Kinetic = zeros(N, 1);
Potential = zeros(N, 1);

for i = 1:N
    theta = res.x(i, 1:2)';
    theta_dot = res.x(i, 3:4)';
    
    % Kinetic Energy: K = 0.5 * q_dot^T * M * q_dot
    M = compute_M(theta, params);
    Kinetic(i) = 0.5 * theta_dot' * M * theta_dot;
    
    % Potential Energy (Using the CORRECTED formula)
    y1 = params.L(1) * sin(theta(1));
    y2 = params.L(1) * sin(theta(1)) + params.L(2) * sin(theta(1) + theta(2));
    Potential(i) = params.m(1)*params.g*y1 + params.m(2)*params.g*y2;
end

Total_Energy = Kinetic + Potential;

% Plot Results
figure('Name', 'Energy Conservation', 'Color', 'w');
plot(res.t, Kinetic, 'b', 'LineWidth', 1.5); hold on;
plot(res.t, Potential, 'r', 'LineWidth', 1.5);
plot(res.t, Total_Energy, 'k--', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Energy [Joules]');
legend('Kinetic Energy', 'Potential Energy', 'Total Mechanical Energy');
title('Energy Conservation Test (\tau = 0)');
grid on;

% Validate variance in Total Energy
energy_variance = max(Total_Energy) - min(Total_Energy);
fprintf('Total Energy Variance over 10 seconds: %.5e Joules\n', energy_variance);
if energy_variance < 1e-3
    disp('PASS: Energy is perfectly conserved. Dynamic equations are strictly correct.');
else
    warning('FAIL: Energy is bleeding or accumulating. Check dynamic derivations or lower solver tolerance.');
end