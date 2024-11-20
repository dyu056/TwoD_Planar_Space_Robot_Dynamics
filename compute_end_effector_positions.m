function end_effector_positions = compute_end_effector_positions(states, t, robot_dynamics_constants)
    % Compute end-effector positions for each state and time value
    %
    % Inputs:
    %   states - Nx5 array of state values [x, y, theta0, theta1, theta2] for N states
    %   t - 1xN array of corresponding time values (optional, not used in this example)
    %   robot_dynamics_constants - Structure containing robot parameters
    %
    % Output:
    %   end_effector_positions - Nx2 array of [x_end, y_end] positions
    
    % Preallocate space for end-effector positions
    num_states = size(states, 1);
    end_effector_positions = zeros(num_states, 2);

    % Loop through each state and compute end-effector position
    for i = 1:num_states
        % Extract the current state
        x = states(i, 1);
        y = states(i, 2);
        theta0 = states(i, 3);
        theta1 = states(i, 4);
        theta2 = states(i, 5);

        % Compute the forward kinematics for the current state
        [~, x_end, y_end] = forward_kinematics(x, y, theta0, theta1, theta2, robot_dynamics_constants);

        % Store the end-effector position
        end_effector_positions(i, :) = [x_end, y_end];
    end
end
