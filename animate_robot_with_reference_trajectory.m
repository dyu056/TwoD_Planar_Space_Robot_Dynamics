function animate_robot_with_reference_trajectory(trajectory, time_data, fixed_timestep, robot_dynamics_constants, reference_trajectory)
    % Interpolate the trajectory based on a fixed timestep
    % Inputs:
    %   trajectory - Nx5 matrix containing [x, y, theta0, theta1, theta2]
    %   time_data - 1xN array containing time for each trajectory point
    %   fixed_timestep - Fixed time step for interpolation
    %   robot_dynamics_constants - Structure containing robot dimensions
    %   reference_trajectory - Mx2 array of [x_ref, y_ref] reference points

    % Extract original time data and states
    t_original = time_data;
    x_original = trajectory(:, 1);
    y_original = trajectory(:, 2);
    theta0_original = trajectory(:, 3);
    theta1_original = trajectory(:, 4); % Initial theta is pi/4 in our system
    theta2_original = trajectory(:, 5);

    % Create new time vector with fixed timestep
    t_interpolated = t_original(1):fixed_timestep:t_original(end);

    % Interpolate trajectory data
    x_interpolated = interp1(t_original, x_original, t_interpolated, 'linear');
    y_interpolated = interp1(t_original, y_original, t_interpolated, 'linear');
    theta0_interpolated = interp1(t_original, theta0_original, t_interpolated, 'linear');
    theta1_interpolated = interp1(t_original, theta1_original, t_interpolated, 'linear');
    theta2_interpolated = interp1(t_original, theta2_original, t_interpolated, 'linear');

    % Combine interpolated trajectory
    trajectory_interpolated = [x_interpolated', y_interpolated', theta0_interpolated', theta1_interpolated', theta2_interpolated'];

    % Animate the robot with the interpolated trajectory and reference
    animate_robot_with_rotating_square_base(trajectory_interpolated, robot_dynamics_constants, reference_trajectory, t_interpolated);
end

function animate_robot_with_rotating_square_base(trajectory, robot_dynamics_constants, reference_trajectory, time_data)
    % Extract robot dimensions
    b0 = robot_dynamics_constants.b0;
    l1 = robot_dynamics_constants.l1;
    l2 = robot_dynamics_constants.l2;
    link_radius = robot_dynamics_constants.link_radius;

    % Create a figure for the animation
    figure;
    axis equal;
    grid on;
    hold on;
    xlabel('X-axis');
    ylabel('Y-axis');
    
    % Set axis limits (adjust based on your workspace)
    xlim([-1, 3]);
    ylim([-2, 2]);

    % Plot the reference trajectory
    if ~isempty(reference_trajectory)
        plot(reference_trajectory(:, 1), reference_trajectory(:, 2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Trajectory');
    end

    % Loop through each trajectory state
    for i = 1:size(trajectory, 1)
        % Extract state variables
        x = trajectory(i, 1);
        y = trajectory(i, 2);
        theta0 = trajectory(i, 3);
        theta1 = trajectory(i, 4);
        theta2 = trajectory(i, 5);

        % Compute joint positions
        % Base (centered at x, y) to first joint
        base_corner_x = x + b0 * cos(theta0); % Upper-right diagonal corner
        base_corner_y = y + b0 * sin(theta0);
        
        % First joint to second joint
        joint1_x = base_corner_x + l1 * cos(theta0 + theta1);
        joint1_y = base_corner_y + l1 * sin(theta0 + theta1);
        
        % Second joint to end effector
        end_effector_x = joint1_x + l2 * cos(theta0 + theta1 + theta2);
        end_effector_y = joint1_y + l2 * sin(theta0 + theta1 + theta2);

        % Plot the robot
        % Clear previous frame
        cla;

        % Draw the reference trajectory again for visibility
        if ~isempty(reference_trajectory)
            plot(reference_trajectory(:, 1), reference_trajectory(:, 2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Trajectory');
        end

        % Draw the 1m x 1m square base centered at (x, y) and rotated by theta0
        draw_rotating_square_base(x, y, 1, theta0);

        % Plot the first link
        plot([base_corner_x, joint1_x], [base_corner_y, joint1_y], 'b-', 'LineWidth', 4); % Base to joint 1
        plot_circle(base_corner_x, base_corner_y, link_radius, 'b'); % Joint 1

        % Plot the second link
        plot([joint1_x, end_effector_x], [joint1_y, end_effector_y], 'g-', 'LineWidth', 4); % Joint 1 to joint 2
        plot_circle(joint1_x, joint1_y, link_radius, 'g'); % Joint 2

        % Plot the end effector
        plot_circle(end_effector_x, end_effector_y, link_radius, 'r'); % End effector

        % Update the title to include current time
        current_time = time_data(i);
        title(sprintf('Robot Arm Animation - Time: %.2f seconds', current_time));

        % Pause to create animation effect
        pause(0.05); % Adjust for smoother animation
    end
end

function draw_rotating_square_base(center_x, center_y, side_length, theta)
    % Draw a square centered at (center_x, center_y) with a given side length
    % and rotated by an angle theta (in radians)

    % Half the side length
    half_side = side_length / 2;

    % Define the unrotated square corners relative to the center
    digonal_length = sqrt(2 * half_side ^2);
    corners = [
        -digonal_length, 0; % Bottom-left
         0, -digonal_length; % Bottom-right
         digonal_length,  0; % Top-right
        0,  digonal_length; % Top-left
    ];

    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Rotate the corners
    rotated_corners = (R * corners')';

    % Translate the corners to the square's center
    rotated_corners(:, 1) = rotated_corners(:, 1) + center_x;
    rotated_corners(:, 2) = rotated_corners(:, 2) + center_y;

    % Close the square by appending the first corner
    rotated_corners = [rotated_corners; rotated_corners(1, :)];

    % Draw the square
    fill(rotated_corners(:, 1), rotated_corners(:, 2), [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'FaceAlpha', 0.5); % Gray square with transparency
end

function plot_circle(x, y, radius, color)
    % Plot a circle to represent joints or end effector
    theta = linspace(0, 2*pi, 100);
    circle_x = radius * cos(theta) + x;
    circle_y = radius * sin(theta) + y;
    fill(circle_x, circle_y, color, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end
