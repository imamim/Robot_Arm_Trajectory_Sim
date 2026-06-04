% 1. Load Physical parameters and Controller Gains
params = get_robot_params();

% 2. Define the Test Scenario
tspan = [0 10];
% Initial Joint State: [theta1; theta2; theta1_dot; theta2_dot; err_int_x; err_int_y]
x0 = [pi/2; -pi/4; 0; 0; 0; 0]; 

% Define the Cartesian trajectory generator (Circle in X-Y plane)
center_x = 0.5;
center_y = 0.5;
radius = 0.2;
omega = 1.0; % rad/s

trajectory_func = @(t) [
    % Desired Position [x; y]
    center_x + radius * cos(omega*t);
    center_y + radius * sin(omega*t);
    % Desired Velocity [x_dot; y_dot]
    -radius * omega * sin(omega*t);
     radius * omega * cos(omega*t);
    % Desired Acceleration [x_ddot; y_ddot]
    -radius * omega^2 * cos(omega*t);
    -radius * omega^2 * sin(omega*t)
];

% 3. Package Options and Run Simulation
options = get_sim_options(tspan, x0, trajectory_func, 1e-5);

% Use the OSC Master Controller!
res = simulateRobot(params, @master_controller_OSC, options);

% 4. Analysis: Evaluate References
N = length(res.t);
x_ref = zeros(N, 2);
x_dot_ref = zeros(N, 2);
x_ddot_ref = zeros(N, 2);
int_term_applied = zeros(N, 2);

for i = 1:N
    traj_state = trajectory_func(res.t(i));
    x_ref(i, :) = traj_state(1:2)';
    x_dot_ref(i, :) = traj_state(3:4)';
    x_ddot_ref(i, :) = traj_state(5:6)';
    
    % Calculate applied integral term (Saturated Error * Ki)
    % res.x(:, 5:6) now holds Cartesian integral error
    int_err = res.x(i, 5:6)';
    int_err_sat = max(min(int_err, params.int_limit), -params.int_limit);
    int_term_applied(i, :) = (params.Ki * int_err_sat)';
end

% 5. Analysis: Plot Cartesian Tracking Performance
figure('Name', 'OSC Analysis - Cartesian Tracking', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 800]);

% --- X Axis ---
% Position
subplot(2,2,1);
plot(res.t, x_ref(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x2, 'b-', 'LineWidth', 1.5); % res.x2 is the end-effector X
title('X-Axis: Position Tracking');
ylabel('Position [m]');
legend('Reference', 'Actual');
grid on;

% Velocity & Integrator
subplot(2,2,3);
yyaxis left;
plot(res.t, x_dot_ref(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x_dot(:, 1), 'b-', 'LineWidth', 1.5);
ylabel('Velocity [m/s]');

yyaxis right;
plot(res.t, int_term_applied(:, 1), 'g-.', 'LineWidth', 1.5);
ylabel('Integral Effort [m/s^2]');

title('X-Axis: Velocity Tracking & Integrator');
legend('Reference Vel', 'Actual Vel', 'Integral Effort', 'Location', 'best');
xlabel('Time [s]');
grid on;

% --- Y Axis ---
% Position
subplot(2,2,2);
plot(res.t, x_ref(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.y2, 'b-', 'LineWidth', 1.5); % res.y2 is the end-effector Y
title('Y-Axis: Position Tracking');
ylabel('Position [m]');
legend('Reference', 'Actual');
grid on;

% Velocity & Integrator
subplot(2,2,4);
yyaxis left;
plot(res.t, x_dot_ref(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.x_dot(:, 2), 'b-', 'LineWidth', 1.5);
ylabel('Velocity [m/s]');

yyaxis right;
plot(res.t, int_term_applied(:, 2), 'g-.', 'LineWidth', 1.5);
ylabel('Integral Effort [m/s^2]');

title('Y-Axis: Velocity Tracking & Integrator');
legend('Reference Vel', 'Actual Vel', 'Integral Effort', 'Location', 'best');
xlabel('Time [s]');
grid on;


% 6. Analysis: Plot Torque Saturation
figure('Name', 'OSC Analysis - Torque Saturation', 'NumberTitle', 'off', 'Position', [150, 150, 800, 600]);

subplot(2,1,1);
plot(res.t, res.tau_cmd(:, 1), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.tau(:, 1), 'b-', 'LineWidth', 1.5);
yline(params.tau_max(1), 'k:', 'LineWidth', 1.5);
yline(-params.tau_max(1), 'k:', 'LineWidth', 1.5);
title('Joint 1: Commanded vs Applied Torque');
ylabel('Torque [Nm]');
legend('Commanded Torque', 'Applied Torque', 'Saturation Limits');
grid on;

subplot(2,1,2);
plot(res.t, res.tau_cmd(:, 2), 'r--', 'LineWidth', 1.5); hold on;
plot(res.t, res.tau(:, 2), 'b-', 'LineWidth', 1.5);
yline(params.tau_max(2), 'k:', 'LineWidth', 1.5);
yline(-params.tau_max(2), 'k:', 'LineWidth', 1.5);
title('Joint 2: Commanded vs Applied Torque');
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('Commanded Torque', 'Applied Torque', 'Saturation Limits');
grid on;

% 7. Visualization
animate_robot(res, params);
