% 1. Load Physical parameters and Controller Gains
params = get_robot_params();

% 2. Define the Test Scenario
tspan = [0 10];
% Initial Joint State: [theta1; theta2; theta1_dot; theta2_dot; err_int_x; err_int_y]
x0 = [pi/2; -pi/4; 0; 0; 0; 0]; 

% =========================================================================
% --- NEW: Cubic Spline Figure-Eight (∞) Trajectory Generator ---
% =========================================================================
center_x = 0.0;
center_y = 0.8;
A_x = 0.5; % Amplitude in X (Horizontal width)
A_y = 0.1; % Amplitude in Y (Vertical height - made smaller for horizontal orientation)
T_cycle = 2.0; % Time to complete one full figure-eight loop (seconds)

% 1. Generate discrete waypoints representing the Figure-8 (Lissajous curve)
% We sample points along the desired time span to act as spline knots.
t_waypoints = linspace(tspan(1), tspan(2), 40); 
x_waypoints = center_x + A_x * sin(2*pi * t_waypoints / T_cycle);
y_waypoints = center_y + A_y * sin(4*pi * t_waypoints / T_cycle); % 2x freq for the '8' shape


% 2. Generate Base Cubic Splines (Position)
% MATLAB's 'spline' creates a piecewise polynomial (pp) structure with C2 continuity
pp_x = spline(t_waypoints, x_waypoints);
pp_y = spline(t_waypoints, y_waypoints);

% 3. Differentiate Splines for Velocity
% A cubic polynomial is ax^3 + bx^2 + cx + d. Its derivative is 3ax^2 + 2bx + c.
% We multiply the first 3 columns of the coefficient matrix by [3, 2, 1].
pp_x_dot = mkpp(pp_x.breaks, pp_x.coefs(:, 1:3) .* [3, 2, 1]);
pp_y_dot = mkpp(pp_y.breaks, pp_y.coefs(:, 1:3) .* [3, 2, 1]);

% 4. Differentiate Splines for Acceleration
% A quadratic polynomial is ax^2 + bx + c. Its derivative is 2ax + b.
% We multiply the first 2 columns of the coefficient matrix by [2, 1].
pp_x_ddot = mkpp(pp_x_dot.breaks, pp_x_dot.coefs(:, 1:2) .* [2, 1]);
pp_y_ddot = mkpp(pp_y_dot.breaks, pp_y_dot.coefs(:, 1:2) .* [2, 1]);

t_interp = linspace(tspan(1), tspan(2), 1000); 

x_sp = ppval(pp_x, t_interp);
y_sp = ppval(pp_y, t_interp);

figure('Name', 'Figure-8 Cubic Spline', 'NumberTitle', 'off');
plot(x_sp, y_sp, 'LineWidth', 1.5,'Color','green');
title('Reference Figure-8');
xlabel('X Position [m]');
ylabel('Y Position [m]');
grid on;
axis equal;

% 5. Define the Trajectory Function Handle
% This dynamically evaluates the splines at any given time 't' during the simulation ODE steps.
trajectory_func = @(t) [
    % Desired Position [x; y]
    ppval(pp_x, t);
    ppval(pp_y, t);
    % Desired Velocity [x_dot; y_dot]
    ppval(pp_x_dot, t);
    ppval(pp_y_dot, t);
    % Desired Acceleration [x_ddot; y_ddot]
    ppval(pp_x_ddot, t);
    ppval(pp_y_ddot, t)
];
% =========================================================================



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
