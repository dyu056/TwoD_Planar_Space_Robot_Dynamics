% Inverse kinematic using iteration method
% Use current position as initial guess to yield the joint state at the
% next moment.

% Can use fsolve to do this straightforwardly but I wish to dive deeper
% into couple of iterative methods hahaha

function [joint_angles] = inverse_kinematic_broyden(x_ref, y_ref, initial_joint_angles, robot_dynamics_constants)
   % Define F(x) = (x - xref)^2 + (y - yref)^2
   joint_angles = initial_joint_angles; %must be [theta1; theta2]
   H = [1 0; 0 1];
   error = F(joint_angles, x_ref, y_ref, robot_dynamics_constants);
   tol = 0.05;
   maxiter = 10000;
   k = 0;
   alpha = 0.01;
   while (get_scalar_error(error) > tol && k <= maxiter)
        direction = -H * F(joint_angles, x_ref, y_ref, robot_dynamics_constants);
        joint_angles_new = joint_angles + alpha * direction;
        error_new = F(joint_angles_new, x_ref, y_ref, robot_dynamics_constants);
        % Update H
        s = joint_angles_new - joint_angles;
        y = error_new - error;
        H = H + (s - H*y)*transpose(s - H*y)/(transpose(s - H*y) * y);
        
        error = error_new;
        joint_angles = joint_angles_new;
        k = k + 1;
   end

   if k >= maxiter
       message = "Max iteration reached, err = " + string(get_scalar_error(error));
       %warning(message)
   end
end

function result = F(joint_angles, xref, yref, robot_dynamics_constants)
    [~, x_end, y_end] = forward_kinematics(0, 0, 0, joint_angles(1), joint_angles(2), robot_dynamics_constants);
    result = [(x_end - xref)^2; (y_end - yref)^2];
end

function scalar_error = get_scalar_error(error_vector)
    scalar_error = sqrt(error_vector(1) + error_vector(2));
end