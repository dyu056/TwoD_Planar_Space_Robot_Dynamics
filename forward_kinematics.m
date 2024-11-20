function [T_inertial_end, x_end, y_end] = forward_kinematics(x, y, theta0, theta1, theta2, robot_dynamics_constants)
    % Defined the forward kinematic of our robot
    T_end = dhMatrix(0, 0, 0, 0); % Represents the end effector position
    T_2_end = dhMatrix(theta2, 0, robot_dynamics_constants.l2, 0);
    T_1_2 = dhMatrix(theta1, 0, robot_dynamics_constants.l1, 0);
    T_body_1 = dhMatrix(theta0, 0, robot_dynamics_constants.b0, 0);
    T_body_end = T_body_1*T_1_2*T_2_end*T_end;
    T_inertial_body = pure_translation(x,y);
    T_inertial_end = T_inertial_body*T_body_end;
    x_end = T_inertial_end(1, 4);
    y_end = T_inertial_end(2, 4);
end

function T = dhMatrix(theta, d, a, alpha)
    % DH Transformation Matrix
    % Computes the homogeneous transformation matrix using DH parameters.
    %
    % Inputs:
    %   theta - Joint angle (in radians)
    %   d     - Link offset (distance along z-axis)
    %   a     - Link length (distance along x-axis)
    %   alpha - Link twist (angle between z-axes in radians)
    %
    % Output:
    %   T     - 4x4 Homogeneous Transformation Matrix

    % Compute the transformation matrix
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

function T_translate = pure_translation(x,y)
    T_translate = [1 0 0 x; 0 1 0 y; 0 0 1 0; 0 0 0 1];
end