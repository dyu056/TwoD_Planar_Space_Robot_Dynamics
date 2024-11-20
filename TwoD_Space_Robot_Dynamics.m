clc; clear;

% I would say it is still the problem of the lagrangian equation. I don't
% think the simscape has got anything wrong.
% Check Coriolis!!!!
%% Define initial conditions for q and q_dot
initial_q = [0; 0; 0; 0; 0];         % Initial joint angles (or positions), e.g., radians or meters
initial_q_dot = [0; 0; 0; 0; 0];     % Initial joint velocities, e.g., radians/s or m/s
initial_state = [initial_q; initial_q_dot]; % Combined state vector

% Define the torque input tau (can be a constant or a function of time)
generalized_force = [0.0; 0.0; 0.0; 0.0; 1.0]; % Constant torques for each joint, for example
torque = generalized_force(4:5);

robot_dynamics_constants.m0 = 500;
robot_dynamics_constants.m1 = 10;
robot_dynamics_constants.m2 = 10;
robot_dynamics_constants.b0 = sqrt(0.5^2 + 0.5^2);
robot_dynamics_constants.l1 = 1;
robot_dynamics_constants.l2 = 1;
robot_dynamics_constants.Ic0 = 83.3333;
robot_dynamics_constants.Ic1 = 2;
robot_dynamics_constants.Ic2 = 2;

%% Define time span with 0.1 s step
tspan = 0:0.1:10; % From 0 to 10 seconds, with a step of 0.1s

%% Integrate using ode45 with specified output times
[t, state] = ode113(@(t, state) Planar_Space_Robot_Dynamics(t, state, generalized_force, robot_dynamics_constants), tspan, initial_state);

% Extract results
q_results = state(:, 1:5);       % Joint positions over time
q_dot_results = state(:, 6:10);   % Joint velocities over time


%% Test simulink model
% Load the model
load_system('SpaceRobotics_v2.slx')

time = (0:0.01:10)';  % Column vector of time from 0 to 10 seconds in 0.1 increments
torque_values = ones(size(time));  % Constant torque of 1

% Combine time and torque values into one matrix
torque_input = [time, torque_values, torque_values];

%% Run the simulation
simOut = sim('SpaceRobotics_v2', 'StopTime', '10');

% Access outputs
theta0 = simOut.theta0;
theta1 = simOut.theta1;
theta2 = simOut.theta2;
theta0_axis = simOut.theta0_axis;
theta0_direction = reshape(theta0_axis.Data(3,1,:), [], size(theta0_axis.Data(3,1,:), 2));
x = simOut.x;
y = simOut.y;

%testdata=reshape(theta1.Data(1,1,:), [1,55]);
%% Create a 2x3 grid of subplots for organized display
figure;

% Plot Theta0
subplot(2, 3, 1);
plot(theta0.Time, (theta0.Data) .* theta0_direction * 180 / pi);
hold on
plot(t, q_results(:, 3) * 180 / pi);
hold off
title('Theta0');
xlabel('Time');
ylabel('Theta0');
legend('Simscape', 'Lagrangian'); % Add legend

% Plot Theta1
subplot(2, 3, 2);
plot(theta1.Time, (theta1.Data) * 180 / pi);
hold on
plot(t, q_results(:, 4) * 180 / pi);
hold off
title('Theta1');
xlabel('Time');
ylabel('Theta1');
legend('Simscape', 'Lagrangian'); % Add legend

% Plot Theta2
subplot(2, 3, 3);
plot(theta2.Time, (theta2.Data) * 180 / pi);
hold on
plot(t, q_results(:, 5) * 180 / pi);
hold off
title('Theta2');
xlabel('Time');
ylabel('Theta2');
legend('Simscape', 'Lagrangian'); % Add legend

% Plot x
subplot(2, 3, 4);
plot(x.Time, x.Data);
hold on
plot(t, q_results(:, 1));
hold off
title('x');
xlabel('Time');
ylabel('x');
legend('Simscape', 'Lagrangian'); % Add legend

% Plot y
subplot(2, 3, 5);
plot(y.Time, y.Data);
hold on
plot(t, q_results(:, 2));
hold off
title('y');
xlabel('Time');
ylabel('y');
legend('Simscape', 'Lagrangian'); % Add legend

% Adjust layout for better visibility
sgtitle('Joint Angles and Positions Over Time'); % Optional overarching title
