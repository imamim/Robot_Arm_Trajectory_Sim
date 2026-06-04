function [x_ddot_cmd, err, err_dot] = ctrl_outer_PID_OSC(x_d, x_d_dot, x_d_ddot, x, x_dot, err_int, params)
% CTRL_OUTER_PID_OSC Calculates the commanded virtual Cartesian acceleration.
%
% Inputs:
%   x_d        : mx1 desired Cartesian positions (from cubic spline)
%   x_d_dot    : mx1 desired Cartesian velocities
%   x_d_ddot   : mx1 desired Cartesian accelerations (feedforward)
%   x          : mx1 actual Cartesian positions (from Forward Kinematics)
%   x_dot      : mx1 actual Cartesian velocities (from J * q_dot)
%   err_int    : mx1 accumulated integral of Cartesian position error
%   params     : struct containing controller gains (Kp, Kd, Ki) 
%                and anti-windup limit (int_limit)
%
% Outputs:
%   x_ddot_cmd : mx1 commanded Cartesian acceleration (a_x) to feed to inner loop
%   err        : mx1 current Cartesian position error (to be integrated)
%   err_dot    : mx1 current Cartesian velocity error

    % 1. Calculate state errors in Task Space
    err = x_d - x;
    err_dot = x_d_dot - x_dot;
    
    % 2. Anti-Windup Protection for Integral Term
    % Prevents the integrator from winding up if the robot gets stuck on an
    % obstacle or reaches a joint limit.
    current_int_term = err_int;
    for i = 1:length(current_int_term)
        if current_int_term(i) > params.int_limit
            current_int_term(i) = params.int_limit;
        elseif current_int_term(i) < -params.int_limit
            current_int_term(i) = -params.int_limit;
        end
    end
    
    % 3. Calculate Commanded Cartesian Acceleration (a_x)
    % Form: x_ddot_d + Kp*e + Kd*e_dot + Ki*int(e)
    x_ddot_cmd = x_d_ddot ...
                 + params.Kp * err ...
                 + params.Kd * err_dot ...
                 + params.Ki * current_int_term;
                 
end