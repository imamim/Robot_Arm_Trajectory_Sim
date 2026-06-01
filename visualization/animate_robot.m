function animate_robot(results, p)
% ANIMATE_ROBOT Visualizes the 2-DoF manipulator motion in real-time.
%
% Inputs:
%   results - Struct returned by simulateRobot()
%   p       - Parameter structure containing link lengths

    % 1. Extract variables from results
    t = results.t;
    x = results.x;
    
    L1 = p.L(1);
    L2 = p.L(2);
    
    theta1 = x(:, 1);
    theta2 = x(:, 2);
    num_frames = length(t);

    % 2. Setup Animation Figure
    fig = figure('Name', '2-DoF Manipulator Animation', 'Color', 'w');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    
    % Set bounding box limits based on maximum reach
    max_reach = (L1 + L2) * 1.2;
    xlim(ax, [-max_reach, max_reach]);
    ylim(ax, [-max_reach, max_reach]);
    xlabel(ax, 'X Position [m]');
    ylabel(ax, 'Y Position [m]');
    title(ax, 'Robot Workspace Animation');

    % 3. Initialize Graphic Elements (Handles)
    % Base joint (Anchor point at origin)
    plot(ax, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    
    % Links (Line objects)
    link1_line = plot(ax, [0, 0], [0, 0], 'LineWidth', 4);
    link2_line = plot(ax, [0, 0], [0, 0], 'LineWidth', 4);
    
    % Joint/End-Effector (Scatter/Marker objects)
    joint2_dot = plot(ax, 0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    ee_dot     = plot(ax, 0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    
    % End-Effector Trajectory Trail
    trail_line = plot(ax, NaN, NaN, 'r:', 'LineWidth', 1);
    ee_x_history = zeros(num_frames, 1);
    ee_y_history = zeros(num_frames, 1);

    % 4. Pre-compute Forward Kinematics for all frames (Vectorized)
    % Coordinates of Joint 2 (Elbow)
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    
    % Coordinates of End-Effector (Wrist/Mass 2)
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);

    % 5. Animation Playback Loop
    disp('Playing animation...');
    tic;
    for k = 1:num_frames
        % Check if user closed the figure window prematurely
        if ~ishandle(fig), break; end
        
        % Update link lines data
        set(link1_line, 'XData', [0, x1(k)], 'YData', [0, y1(k)]);
        set(link2_line, 'XData', [x1(k), x2(k)], 'YData', [y1(k), y2(k)]);
        
        % Update mass/joint markers
        set(joint2_dot, 'XData', x1(k), 'YData', y1(k));
        set(ee_dot,     'XData', x2(k), 'YData', y2(k));
        
        % Update trajectory trail
        ee_x_history(k) = x2(k);
        ee_y_history(k) = y2(k);
        set(trail_line, 'XData', ee_x_history(1:k), 'YData', ee_y_history(1:k));
        
        % Force MATLAB to flush graphics queue to sync with simulation time
        if k < num_frames
            % Calculate how much real time should be paused to match simulation time
            expected_time = t(k+1) - t(k);
            drawnow;
            pause(expected_time);
        end
    end
    disp('Animation playback completed.');
end