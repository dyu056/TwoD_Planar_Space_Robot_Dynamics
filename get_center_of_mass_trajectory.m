%% Compute trajectory of center of mass with state trajectory
function [rg_trajectory] = get_center_of_mass_trajectory(state_trajectory, robot_dynamics_constants)
    % Input:
    %   state_trajectory: Nx5 matrix where each row is [xb, yb, theta0, theta1, theta2]
    %   robot_dynamics_constants: Struct with robot constants
    % Output:
    %   rg_trajectory: Nx2 matrix where each row is the center of mass [rgx, rgy] at each time step

    % Number of time steps
    num_steps = size(state_trajectory, 1);

    % Initialize trajectory output
    rg_trajectory = zeros(num_steps, 2);

    % Loop through each state in the trajectory
    for i = 1:num_steps
        % Extract the current state
        state = state_trajectory(i, :);

        % Compute the center of mass for the current state
        rg = get_center_of_mass(state, robot_dynamics_constants);

        % Store the result
        rg_trajectory(i, :) = rg';
    end
end

%% Get center of mass with state
function [rg] = get_center_of_mass(state, robot_dynamics_constants)
    % Extract state variables
    xb = state(1);
    yb = state(2);
    theta0 = state(3);
    theta1 = state(4);
    theta2 = state(5);

    % Extract robot dynamics constants
    b0 = robot_dynamics_constants.b0;
    l1 = robot_dynamics_constants.l1;
    l2 = robot_dynamics_constants.l2;

    m0 = robot_dynamics_constants.m0;
    m1 = robot_dynamics_constants.m1;
    m2 = robot_dynamics_constants.m2;
    M = m0 + m1 + m2; % Total mass
    
    static = 0;
    % Compute positions
    rb = [xb; yb]; % Base position
    r1 = rb + rotation(theta0 + static) * [b0; 0] ...
            + rotation(theta0 + static + theta1) * [l1/2; 0];
    r2 = rb + rotation(theta0 + static) * [b0; 0] ...
            + rotation(theta0 + static + theta1) * [l1; 0] ...
            + rotation(theta0 + static + theta1 + theta2) * [l2/2; 0];

    % Compute center of mass
    rg = (rb * m0 + r1 * m1 + r2 * m2) / M;
end

%% Rotation matrix function
function [R] = rotation(theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end
