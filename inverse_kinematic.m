function inverse_kinematics_solution = inverse_kinematic(target_x, target_y, initial_guess, robot_dynamics_constants)
    % Define the system of equations for inverse kinematics
    equations = @(variables) forward_kinematics_error(variables, target_x, target_y, robot_dynamics_constants);

    % Set options for fsolve
    options = optimset('Display', 'off'); % Display iterations

    % Solve the equations using fsolve
    inverse_kinematics_solution = fsolve(equations, initial_guess, options);
end

function errors = forward_kinematics_error(variables, target_x, target_y, robot_dynamics_constants)
    % Extract variables
    theta1 = variables(1);
    theta2 = variables(2);

    % Compute the forward kinematics
    [~, x_end, y_end] = forward_kinematics(0, 0, 0, theta1, theta2, robot_dynamics_constants);

    % Define the error between computed and target end-effector position
    errors = [abs(x_end - target_x); abs(y_end - target_y)];
end
