%% Get center of mass with state
function [xend, yend] = forward_kinematic_naive(state, robot_dynamics_constants)
    xb = state(1);
    yb = state(2);
    theta0 = state(3);
    theta1 = state(4);
    theta2 = state(5);

    b0 = robot_dynamics_constants.b0;
    l1 = robot_dynamics_constants.l1;
    l2 = robot_dynamics_constants.l2;
    
    rb = [xb; yb];
    re = rb + rotation(theta0 + pi/4)*[b0; 0] + rotation(theta0 + pi/4 + theta1)*[l1; 0] + rotation(theta0 + pi/4 + theta1 + theta2)*[l2; 0];
    xend = re(1);
    yend = re(2);
end

function [R] = rotation(theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end
