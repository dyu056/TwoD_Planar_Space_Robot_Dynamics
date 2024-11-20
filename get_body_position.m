%% Get base position rb from rg and angles
function [rb] = get_body_position(rg, theta0, theta1, theta2, robot_dynamics_constants)
    % Extract robot dynamics constants
    b0 = robot_dynamics_constants.b0;
    l1 = robot_dynamics_constants.l1;
    l2 = robot_dynamics_constants.l2;

    m0 = robot_dynamics_constants.m0;
    m1 = robot_dynamics_constants.m1;
    m2 = robot_dynamics_constants.m2;
    M = m0 + m1 + m2; % Total mass

    % Rotational terms for r1 and r2
    static = 0;
    rotation_0 = rotation(theta0 + static);
    rotation_1 = rotation(theta0 + static + theta1);
    rotation_2 = rotation(theta0 + static + theta1 + theta2);

    % Compute r1 and r2 (without rb)
    r1_offset = rotation_0 * [b0; 0] + rotation_1 * [l1/2; 0];
    r2_offset = rotation_0 * [b0; 0] + rotation_1 * [l1; 0] + rotation_2 * [l2/2; 0];

    % Mass contributions for r1 and r2
    r1_contribution = r1_offset * m1;
    r2_contribution = r2_offset * m2;

    % Compute rb
    rb = (rg * M - (r1_contribution + r2_contribution)) / M;
end

% Rotation matrix function
function [R] = rotation(theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end
