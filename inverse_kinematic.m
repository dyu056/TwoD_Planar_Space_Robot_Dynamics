function inverse_kinematics_solution = inverse_kinematic(target_x, target_y, initial_guess, robot_dynamics_constants, center_of_mass)
    % Define the system of equations for inverse kinematics
    equations = @(variables) forward_kinematics_error(variables, target_x, target_y, robot_dynamics_constants, center_of_mass);

    % Set options for fsolve
    options = optimset('Display', 'off'); % Display iterations

    % Solve the equations using fsolve
    inverse_kinematics_solution = fsolve(equations, initial_guess, options);
end

function errors = forward_kinematics_error(variables, target_x, target_y, robot_dynamics_constants, center_of_mass)
    % Extract variables, we assume that theta0 is constant (which is to be
    % controlled
    theta1 = variables(1);
    theta2 = variables(2);

    % Need to yield new body mass position base on new theta, we know that
    % center of mass should be constant!
    [rb] = get_body_position(center_of_mass, 0, theta1, theta2, robot_dynamics_constants);

    % Compute body position base on center of mass position and 3 joint
    % angles

    % Compute the forward kinematics
    [~, x_end, y_end] = forward_kinematics(rb(1), rb(2), 0, theta1, theta2, robot_dynamics_constants);

    % Define the error between computed and target end-effector position
    errors = [abs(x_end - target_x); abs(y_end - target_y)];
end