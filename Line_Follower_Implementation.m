clc; clear;
%% Initial setup
initial_q = [0; 0; 0; 0; 0];         % Initial joint angles (or positions), e.g., radians or meters
initial_q_dot = [0; 0; 0; 0; 0];     % Initial joint velocities, e.g., radians/s or m/s
initial_state = [initial_q; initial_q_dot]; % Combined state vector

robot_dynamics_constants.m0 = 500;
robot_dynamics_constants.m1 = 10;
robot_dynamics_constants.m2 = 10;
robot_dynamics_constants.b0 = sqrt(0.5^2 + 0.5^2);
robot_dynamics_constants.l1 = 1;
robot_dynamics_constants.l2 = 1;
robot_dynamics_constants.Ic0 = 83.3333;
robot_dynamics_constants.Ic1 = 2;
robot_dynamics_constants.Ic2 = 2;

[center_of_mass] = get_center_of_mass(initial_q, robot_dynamics_constants)

% test
[rb] = get_body_position(center_of_mass, initial_q(3), initial_q(4), initial_q(5), robot_dynamics_constants)

%% Forward kinematic test
test = initial_q;
test = [0; 0; 0; 0; 0];
[T_inertial_end, x_end, y_end] = forward_kinematics(test(1), test(2), test(3), test(4), test(5), robot_dynamics_constants);
[xend_naive, yend_naive] = forward_kinematic_naive(test, robot_dynamics_constants);
disp(T_inertial_end);
disp(xend_naive)
disp(yend_naive)
disp(x_end);
disp(y_end);

%% Guidance trajectory setup
% Mission: Approaching the axis horizontally and then approach vertically
% Mission sequence:
% 1: Define trajectories to follow ahead. Here I would use a point to
% represent it.
% 2: Store it on your computer 
% 3: Follow this array of points.
[T_inertial_end, x_end, y_end] = forward_kinematics(initial_q(1), initial_q(2), initial_q(3), initial_q(4), initial_q(5), robot_dynamics_constants);
start_pos = [x_end, y_end];
%line_start = [0, y_end];
%line_end = [0, 2.4];

% Key points (2D space)
%key_points = [start_pos; line_start; line_end];
key_points = [start_pos; [2, 1]; [1,1]; [1,0.4]; start_pos];
% Number of points to generate between each key point
num_points = 20;
% Generate the trajectory
trajectory = simple_trajectory_generator(key_points, num_points);
joint_angle_trajectories = [];
initial_guess = [0;0];

% Form doable joint angle list
for i = 1:size(trajectory, 1)
    % Get the current point
    current_point = trajectory(i, :);

    % Display the point
    fprintf('Point %d: (%.2f, %.2f)\n', i, current_point(1), current_point(2));

    % Add any operations you'd like to perform on each point here
    [joint_angles] = inverse_kinematic(current_point(1), current_point(2), initial_guess, robot_dynamics_constants, center_of_mass);
    joint_angle_trajectories = [joint_angle_trajectories, joint_angles];
    initial_guess = joint_angles;
end
joint_angle_trajectories = transpose(joint_angle_trajectories);
%% Verify
end_effector_pos = [];
for i = 1:size(joint_angle_trajectories, 1)
    % Get the current point
    current_point = joint_angle_trajectories(i, :);
    [rb] = get_body_position(center_of_mass, 0, current_point(1), current_point(2), robot_dynamics_constants);
    % Add any operations you'd like to perform on each point here
    [~, x_end, y_end] = forward_kinematics(rb(1), rb(2), 0, current_point(1), current_point(2), robot_dynamics_constants);
    end_effector_pos = [end_effector_pos; [x_end, y_end]];
end

%% Plot the trajectory
figure;
plot(trajectory(:, 1), trajectory(:, 2), '-o');
hold on;
plot(key_points(:, 1), key_points(:, 2), 'r*', 'MarkerSize', 10);
plot(end_effector_pos(:, 1), end_effector_pos(:, 2), '-x');
legend('Trajectory', 'Key Points', 'End effector actual');
xlabel('X');
ylabel('Y');
title('Generated Straight-Line Trajectory');
grid on;
axis equal

%% Plot joint angle trajectory
figure;
plot(joint_angle_trajectories(:, 1), joint_angle_trajectories(:, 2), '-o');
xlabel('theta1');
ylabel('theta2');
title('Generated joint angle Trajectory');
grid on;
axis equal

%% Start the mission
clc
% Define time span with 0.1 s step
tspan = 0:0.1:500; % From 0 to 10 seconds, with a step of 0.1s
% Integrate using ode45 with specified output times

global index; % Index for goal points
index = 1;

control_constant.Kp = 50;
control_constant.Kd = 50;

joint_angles_list = joint_angle_trajectories;

[t, state] = ode113(@(t, state) Planar_Space_Robot_Dynamics_with_Control(t, state, robot_dynamics_constants, joint_angles_list, control_constant), tspan, initial_state);

% Extract results
q_results = state(:, 1:5);       % Joint positions over time
q_dot_results = state(:, 6:10);   % Joint velocities over time

%% Create a 2x3 grid of subplots for organized display
figure;

% Plot Theta0
subplot(2, 3, 1);
plot(t, q_results(:, 3));
title('Theta0');
xlabel('Time');
ylabel('Theta0');

% Plot Theta1
subplot(2, 3, 2);
plot(t, q_results(:, 4));
title('Theta1');
xlabel('Time');
ylabel('Theta1');

% Plot Theta2
subplot(2, 3, 3);
plot(t, q_results(:, 5));
title('Theta2');
xlabel('Time');
ylabel('Theta2');

% Plot x
subplot(2, 3, 4);
plot(t, q_results(:, 1));
title('x');
xlabel('Time');
ylabel('x');

% Plot y
subplot(2, 3, 5);
plot(t, q_results(:, 2));
title('y');
xlabel('Time');
ylabel('y');

% Adjust layout for better visibility
sgtitle('Joint Angles and Positions Over Time'); % Optional overarching title

% Compute end effector positions
end_effector_positions = compute_end_effector_positions(q_results, t, robot_dynamics_constants);
x_end = end_effector_positions(:, 1);
y_end = end_effector_positions(:, 2);

% Create figure for subplots
figure;

% Subplot 1: Time vs X
subplot(3, 1, 1); % Change to 3 rows for an additional subplot
plot(t, x_end, 'LineWidth', 2);
xlabel('Time (t)');
ylabel('X_{end}');
title('Time vs X_{end}');
grid on;

% Subplot 2: Time vs Y
subplot(3, 1, 2);
plot(t, y_end, 'LineWidth', 2);
xlabel('Time (t)');
ylabel('Y_{end}');
title('Time vs Y_{end}');
grid on;

% Subplot 3: Trajectory Covered (X vs Y)
subplot(3, 1, 3);
plot(x_end, y_end, 'LineWidth', 2);
hold on 
plot(trajectory(:, 1), trajectory(:, 2), '-o');
hold off
xlabel('X_{end}');
ylabel('Y_{end}');
title('Trajectory Covered (X vs Y)');
legend("End effector position","Reference")
grid on;
axis equal; % Ensure equal scaling for X and Y axes


%% Animate the robot with interpolated trajectory and reference trajectory
% Fixed timestep
fixed_timestep = 0.1; % 0.1 seconds

robot_dynamics_constants.link_radius = 0.12;
% Animate the robot with interpolated trajectory and reference trajectory
animate_robot_with_reference_trajectory(state, t, fixed_timestep, robot_dynamics_constants, trajectory);

%% Get COM trajectory (Should be one dot for correct kinematic and dynamic)
rg_trajectory = get_center_of_mass_trajectory(state, robot_dynamics_constants);

figure
plot(rg_trajectory(:,1), rg_trajectory(:,2),'o');
xlabel("X")
ylabel("Y")
title("COM position")
xlim([-1,1])
ylim([-1,1])

%% Test forward kinematic

% Codes below are for testing purposes.


test = initial_q;
test = [1; 0; 3; 2; 1];
[T_inertial_end, x_end, y_end] = forward_kinematics(test(1), test(2), test(3), test(4), test(5), robot_dynamics_constants);
[xend_naive, yend_naive] = forward_kinematic_naive(test, robot_dynamics_constants);
disp(T_inertial_end);
disp(xend_naive)
disp(yend_naive)
disp(x_end);
disp(y_end);

%% Test inverse kinematic
clc;
target_pos = [0.5; 0.4];
[joint_angles] = inverse_kinematic(target_pos(1), target_pos(2), [0;0], robot_dynamics_constants, rg);

% Forward verify
test = [0; 0; 0; joint_angles(1); joint_angles(2)];
[T_inertial_end, x_end, y_end] = forward_kinematics(test(1), test(2), test(3), test(4), test(5), robot_dynamics_constants);

disp("fucker")
disp(x_end);
disp(y_end);
disp(joint_angles)
