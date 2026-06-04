% 1. Load Physical parameters and Controller Gains
params = get_robot_params();


% 2. Define the Test Scenario (The "Outside")
tspan = [0 10];
x0 = [pi/2; 0; 0; 0; 0; 0]; 

% Define the trajectory generator
omega = 1.0; % rad/s
trajectory_func = @(t) [
    sin(omega*t); deg2rad(30);          % Desired Position
    omega*cos(omega*t); 0;    % Desired Velocity
    -omega^2*sin(omega*t); 0  % Desired Acceleration
];

% 3. Package Options and Run Simulation
options = get_sim_options(tspan, x0, trajectory_func, 1e-5);

res = simulateRobot(params, @master_controller, options);

% 5. Analysis: Plot Tracking Performance (Position, Velocity, Acceleration)
% Evaluate trajectory over the simulation time to get the reference points
% and re-calculate actual accelerations using forward dynamics.
N = length(res.t);
q_ref = zeros(N, 2);
q_dot_ref = zeros(N, 2);
q_ddot_ref = zeros(N, 2);
q_ddot_act = zeros(N, 2);

int_term_applied = zeros(N, 2);
x_dot_ref = zeros(N, 2);

for i = 1:N
    % Extract references
    traj_state = trajectory_func(res.t(i));
    q_ref(i, :) = traj_state(1:2)';
    q_dot_ref(i, :) = traj_state(3:4)';
    q_ddot_ref(i, :) = traj_state(5:6)';
    
    % Re-calculate actual accelerations
    dx = forward_dynamics(res.t(i), res.x(i, 1:4)', res.tau(i, :)', params);
    q_ddot_act(i, :) = dx(3:4)';
    
    % Calculate applied integral term (Saturated Error * Ki)
    int_err = res.x(i, 5:6)';
    int_err_sat = max(min(int_err, params.int_limit), -params.int_limit);
    int_term_applied(i, :) = (params.Ki * int_err_sat)';
    
    % Calculate Reference Cartesian Velocity
    J_ref = get_jacobian(q_ref(i, :)', params.L(1), params.L(2));
    x_dot_ref(i, :) = (J_ref * q_dot_ref(i, :)')';
end

figure('Name', 'Controller Analysis - Comprehensive Tracking', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 800]);

% --- Joint 1 ---
% Position
subplot(3,2,1);
plot(res.t, q_ref(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x(:, 1), 'b-', 'LineWidth', 1.5);
title('Joint 1: Position Tracking');
ylabel('Position [rad]');
legend('Reference', 'Actual');
grid on;

% Velocity & Integrator
subplot(3,2,3);
yyaxis left;
plot(res.t, q_dot_ref(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x(:, 3), 'b-', 'LineWidth', 1.5);
ylabel('Velocity [rad/s]');

yyaxis right;
plot(res.t, int_term_applied(:, 1), 'g-.', 'LineWidth', 1.5);
ylabel('Integral Effort [rad/s^2]');

title('Joint 1: Velocity Tracking & Integrator');
legend('Reference Vel', 'Actual Vel', 'Integral Effort', 'Location', 'best');
grid on;

% Acceleration
subplot(3,2,5);
plot(res.t, q_ddot_ref(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, q_ddot_act(:, 1), 'b-', 'LineWidth', 1.5);
title('Joint 1: Acceleration Tracking');
xlabel('Time [s]');
ylabel('Accel [rad/s^2]');
grid on;

% --- Joint 2 ---
% Position
subplot(3,2,2);
plot(res.t, q_ref(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x(:, 2), 'b-', 'LineWidth', 1.5);
title('Joint 2: Position Tracking');
ylabel('Position [rad]');
legend('Reference', 'Actual');
grid on;

% Velocity & Integrator
subplot(3,2,4);
yyaxis left;
plot(res.t, q_dot_ref(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x(:, 4), 'b-', 'LineWidth', 1.5);
ylabel('Velocity [rad/s]');

yyaxis right;
plot(res.t, int_term_applied(:, 2), 'g-.', 'LineWidth', 1.5);
ylabel('Integral Effort [rad/s^2]');

title('Joint 2: Velocity Tracking & Integrator');
legend('Reference Vel', 'Actual Vel', 'Integral Effort', 'Location', 'best');
grid on;

% Acceleration
subplot(3,2,6);
plot(res.t, q_ddot_ref(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, q_ddot_act(:, 2), 'b-', 'LineWidth', 1.5);
title('Joint 2: Acceleration Tracking');
xlabel('Time [s]');
ylabel('Accel [rad/s^2]');
grid on;

% 6. Analysis: Plot Torque Saturation
figure('Name', 'Controller Analysis - Torque Saturation', 'NumberTitle', 'off', 'Position', [150, 150, 800, 600]);

% Joint 1 Torque
subplot(2,1,1);
plot(res.t, res.tau_cmd(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.tau(:, 1), 'b-', 'LineWidth', 1.5);
yline(params.tau_max(1), 'k:', 'LineWidth', 1.5);
yline(-params.tau_max(1), 'k:', 'LineWidth', 1.5);
title('Joint 1: Commanded vs Applied Torque');
ylabel('Torque [Nm]');
legend('Commanded Torque', 'Applied (Saturated) Torque', 'Saturation Limits');
grid on;

% Joint 2 Torque
subplot(2,1,2);
plot(res.t, res.tau_cmd(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.tau(:, 2), 'b-', 'LineWidth', 1.5);
yline(params.tau_max(2), 'k:', 'LineWidth', 1.5);
yline(-params.tau_max(2), 'k:', 'LineWidth', 1.5);
title('Joint 2: Commanded vs Applied Torque');
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('Commanded Torque', 'Applied (Saturated) Torque', 'Saturation Limits');
grid on;


% 7. Analysis: Plot Cartesian Velocity Tracking
figure('Name', 'Controller Analysis - Cartesian Velocity', 'NumberTitle', 'off', 'Position', [200, 200, 800, 600]);

% X-Velocity
subplot(2,1,1);
plot(res.t, x_dot_ref(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x_dot(:, 1), 'b-', 'LineWidth', 1.5);
title('End-Effector: X-Axis Velocity Tracking');
ylabel('Velocity [m/s]');
legend('Reference \dot{x}', 'Actual \dot{x}');
grid on;

% Y-Velocity
subplot(2,1,2);
plot(res.t, x_dot_ref(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x_dot(:, 2), 'b-', 'LineWidth', 1.5);
title('End-Effector: Y-Axis Velocity Tracking');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('Reference \dot{y}', 'Actual \dot{y}');
grid on;



% 4. Visualization
% (Remember to only pass the first two columns to the animator)
animate_robot(res, params);


