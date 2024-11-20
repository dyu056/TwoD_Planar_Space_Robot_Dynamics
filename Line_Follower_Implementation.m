clc; clear;
%% Goal 1: Successfully track a point!!!!
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

control_constant.Kp = 50;
control_constant.Kd = 50;

%% Test forward kinematic
test = initial_q;
[T_inertial_end, x_end, y_end] = forward_kinematics(test(1), test(2), test(3), test(4), test(5), robot_dynamics_constants);
disp(T_inertial_end);
disp(x_end);
disp(y_end);

%% Test inverse kinematic
clc;
target_pos = [0.5; 0.4];
[joint_angles] = inverse_kinematic(target_pos(1), target_pos(2), [0;0], robot_dynamics_constants);

% Forward verify
test = [0; 0; 0; joint_angles(1); joint_angles(2)];
[T_inertial_end, x_end, y_end] = forward_kinematics(test(1), test(2), test(3), test(4), test(5), robot_dynamics_constants);

disp("fucker")
disp(x_end);
disp(y_end);
disp(joint_angles)
%% Calculation
% Mission sequence:
% 1: Define trajectories to follow ahead. Here I would use a point to
% represent it.
% 2: Store it on your computer 
% 3: Follow this array of points.

% With point input, calculate joint angle needed
target_pos = [0.5; 0.4];
[joint_angles] = inverse_kinematic(target_pos(1), target_pos(2), [0;0], robot_dynamics_constants);
reference = joint_angles;

% Define time span with 0.1 s step
tspan = 0:0.1:10; % From 0 to 10 seconds, with a step of 0.1s
% Integrate using ode45 with specified output times
[t, state] = ode113(@(t, state) Planar_Space_Robot_Dynamics_with_Control(t, state, robot_dynamics_constants, reference, control_constant), tspan, initial_state);

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

%% Plot out end effector position
end_effector_positions = compute_end_effector_positions(q_results, t, robot_dynamics_constants);
x_end = end_effector_positions(:, 1);
y_end = end_effector_positions(:, 2);
% Create subplots
figure;

% Subplot 1: Time vs X
subplot(2, 1, 1);
plot(t, x_end, 'LineWidth', 2);
xlabel('Time (t)');
ylabel('X_{end}');
title('Time vs X_{end}');
grid on;

% Subplot 2: Time vs Y
subplot(2, 1, 2);
plot(t, y_end, 'LineWidth', 2);
xlabel('Time (t)');
ylabel('Y_{end}');
title('Time vs Y_{end}');
grid on;