function [theta_ddot_cmd, err, err_dot] = ctrl_outer_PID(theta_d, theta_d_dot, theta_d_ddot, theta, theta_dot, err_int, params)
% CTRL_OUTER_PID Calculates the commanded virtual acceleration.
%
% Inputs:
%   theta_d      : nx1 desired joint positions
%   theta_d_dot  : nx1 desired joint velocities
%   theta_d_ddot : nx1 desired joint accelerations (feedforward)
%   theta        : nx1 actual joint positions
%   theta_dot    : nx1 actual joint velocities
%   err_int      : nx1 accumulated integral of position error
%   params       : struct containing controller gains (Kp, Kd, Ki) 
%                  and anti-windup limit (int_limit)
%
% Outputs:
%   theta_ddot_cmd : nx1 commanded joint acceleration
%   err            : nx1 current position error (to be integrated by ODE solver)
%   err_dot        : nx1 current velocity error

    % 1. Calculate state errors
    err = theta_d - theta;
    err_dot = theta_d_dot - theta_dot;

    % 2. Anti-Windup Protection for Integral Term
    % If the magnitude of the integral error exceeds our limit, we clamp it.
    % This prevents the integrator from "winding up" if the robot gets stuck.
    current_int_term = err_int;
    for i = 1:length(current_int_term)
        if current_int_term(i) > params.int_limit
            current_int_term(i) = params.int_limit;
        elseif current_int_term(i) < -params.int_limit
            current_int_term(i) = -params.int_limit;
        end
    end

    % 3. Calculate Commanded Acceleration
    % Form: q_ddot_d + Kp*e + Kd*e_dot + Ki*int(e)
    theta_ddot_cmd = theta_d_ddot ...
                     + params.Kp * err ...
                     + params.Kd * err_dot ...
                     + params.Ki * current_int_term;
                 
end
