% test_tracking_validation.m
% PHASE 3 & 4: Validates closed-loop tracking with integrated real-time animation

clc; clear; close all;


disp('--- Running Tracking Validation with Integrated Animation ---');
params = get_robot_params();

%% 2. Define the Initial and Target Configurations
% Start with the robot hanging completely straight down at rest
options.x0 = [-pi/2; 0; 0; 0];
options.tspan = [0 6]; % 6-second simulation window
options.solverTol = 1e-5; 

% Target Setpoint: Command the robot to reach a fully horizontal posture
q_target = [deg2rad(10); deg2rad(20)]; 

%% 3. Controller Gain Tuning Matrices
% Proportional Gains (Stiffness/Spring constant)
Kp = [120,   0; 
        0,  60]; 
      
% Derivative Gains (Damping/Friction constant)
Kd = [25,   0; 
       0,  12]; 

%% 4. Wrap Controller for the Simulation Engine
% We create an anonymous function handle to seamlessly match the expected 
% internal plant signature of simulateRobot.
ctrl_func = @(t, x, p) controller_pd_gravity(t, x, p, q_target, Kp, Kd);

%% 5. Execute Simulation Engine
res = simulateRobot(params, ctrl_func, options);

%% 6. Run Real-Time Animation Playback
% This will open the visualization window and playback the motion derived 
% from the integrated closed-loop controller states.
animate_robot(res, params);

%% 7. Post-Processing: Performance Evaluation Plots
figure('Name', 'Tracking Performance Metrics', 'Color', 'w');

% Joint Position Trajectories vs. Reference Targets
subplot(2,1,1);
plot(res.t, res.x(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(res.t, res.x(:,2), 'r', 'LineWidth', 1.5);
yline(q_target(1), 'b--', 'Target \theta_1', 'LineWidth', 1.2);
yline(q_target(2), 'r--', 'Target \theta_2', 'LineWidth', 1.2);
ylabel('Joint Position [rad]');
title('Closed-Loop Tracking Response');
legend('\theta_1 (Shoulder)', '\theta_2 (Elbow)', 'Location', 'best');
grid on;

% Control Efforts (Actuator Torques)
subplot(2,1,2);
plot(res.t, res.tau(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(res.t, res.tau(:,2), 'r', 'LineWidth', 1.5);
ylabel('Motor Torque [Nm]');
xlabel('Time [s]');
title('Actuator Control Effort (\tau)');
legend('\tau_1 (Shoulder Motor)', '\tau_2 (Elbow Motor)', 'Location', 'best');
grid on;