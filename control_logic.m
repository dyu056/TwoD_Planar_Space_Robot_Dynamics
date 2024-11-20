function [tau] = control_logic(q, q_dot, reference, control_constants, robot_dynamics_constants)
    % Goal: Control the joint angles
    target_theta = reference;
    % Combine torques into output vector
    [tau] = joint_angle_control(q, q_dot, target_theta, control_constants);
end

function [tau] = joint_angle_control(q, q_dot, target_theta, control_constants)
% Goal: Control the angle of each joint while keeping the base at the same
% position
    Kp = control_constants.Kp;
    Kd = control_constants.Kd;

    % Extract joint angles and velocities
    x = q(1);
    y = q(2);
    theta0 = q(3);
    theta1 = q(4);
    theta2 = q(5);
    dx = q_dot(1);
    dy = q_dot(2);
    dtheta0 = q_dot(3);
    dtheta1 = q_dot(4);
    dtheta2 = q_dot(5);

    % Target angles
    x_ref = 0;
    y_ref = 0;
    theta0_ref = 0;
    theta1_ref = target_theta(1);
    theta2_ref = target_theta(2);

    % Compute control torques
    fx = Kp * (x_ref - x) + Kd * (-dx);
    fy = Kp * (y_ref - y) + Kd * (-dy);
    tau0 = Kp * angle_difference(theta0_ref, theta0) + Kd * (-dtheta0);
    tau1 = Kp * angle_difference(theta1_ref, theta1) + Kd * (-dtheta1);
    tau2 = Kp * angle_difference(theta2_ref, theta2) + Kd * (-dtheta2);

    % Combine torques into output vector
    tau = [fx; fy; tau0; tau1; tau2];
end

function diff = angle_difference(angle1, angle2, isDegrees)
% angle_difference - Compute the difference between two angles.
%
% Syntax:
%   diff = angle_difference(angle1, angle2)
%   diff = angle_difference(angle1, angle2, isDegrees)
%
% Inputs:
%   angle1, angle2 - Input angles to compare. These can be in radians or degrees.
%   isDegrees (optional) - Boolean flag. If true, angles are treated as degrees.
%                          Default is false (angles in radians).
%
% Output:
%   diff - The angular difference, normalized to the range:
%          [-pi, pi] for radians or [-180, 180] for degrees.

    if nargin < 3
        isDegrees = false; % Default to radians
    end

    % Compute raw difference
    rawDiff = angle1 - angle2;

    if isDegrees
        % Normalize to [-180, 180] for degrees
        diff = mod(rawDiff + 180, 360) - 180;
    else
        % Normalize to [-pi, pi] for radians
        diff = mod(rawDiff + pi, 2*pi) - pi;
    end
end

% Experimental: Point tracker scheme
function [tau] = point_tracker(q, q_dot, target_point, control_constants, robot_dynamics_constants)
    % Goal: Control the joint angles to track a point;
    initial_guess = q(4:5);
    [joint_angles] = inverse_kinematic(target_point(1), target_point(2), initial_guess, robot_dynamics_constants);
    %[~, x_end, y_end] = forward_kinematics(0, 0, 0, joint_angles(1), joint_angles(2), robot_dynamics_constants);
    %disp(joint_angles)
    %disp([x_end, y_end])
    % Combine torques into output vector
    [tau] = joint_angle_control(q, q_dot, joint_angles, control_constants);
end
