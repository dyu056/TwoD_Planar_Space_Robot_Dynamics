function d_state = Planar_Space_Robot_Dynamics_with_Control(t, state,  robot_dynamics_constants, joint_angles_list, control_constants)
    normalized_theta0 = mod(state(3), 2*pi);
    normalized_theta1 = mod(state(4), 2*pi);
    normalized_theta2 = mod(state(5), 2*pi);
    q = state(1:5);
    q(3) = normalized_theta0;
    q(4) = normalized_theta1;
    q(5) = normalized_theta2;
    q_dot = state(6:10);

    [reference] = update_reference(joint_angles_list, [q(4),q(5)]);
    
    [inertial_term_matrix, coriolis_term] = Get_Planar_Dynamic_Matrix(q, q_dot, robot_dynamics_constants);
    
    tau = control_logic(q, q_dot, reference, control_constants, robot_dynamics_constants);
    %Dq = tau - coriolis_term;
    Dq = tau - coriolis_term;

    q_ddot = inertial_term_matrix \ Dq;

    d_state = [q_dot; q_ddot];

end

function [reference] = update_reference(joint_angles_list, current_joint_angle)
    global index;
    differences_joint_1 = angle_difference(current_joint_angle(1), joint_angles_list(index, 1), 0);
    differences_joint_2 = angle_difference(current_joint_angle(2), joint_angles_list(index, 2), 0);
    index_length = size(joint_angles_list, 1);
    if is_close(differences_joint_1, differences_joint_2)
        if index + 1 >= index_length
            index = index_length;
        else
            index = index + 1;
        end
    end
    reference = joint_angles_list(index, :);
end

function result = is_close(differences_joint_1, differences_joint_2)
    if (norm([differences_joint_1; differences_joint_2], 2)  <  5e-3)
        result = true;
    else
        result = false;
    end
end


function diff = angle_difference(angle1, angle2, isDegrees)
% angle_difference - Compute the difference between two angles.
%
% Syntax:
%   diff = angle_difference(angle1, angle2)
%   diff = angle_difference(angle1, angle2, isDegrees)
%
% Inputs:
%   angle1, angle2 - Input angles to compare. These can be in radians or degrees.
%   isDegrees (optional) - Boolean flag. If true, angles are treated as degrees.
%                          Default is false (angles in radians).
%
% Output:
%   diff - The angular difference, normalized to the range:
%          [-pi, pi] for radians or [-180, 180] for degrees.

    if nargin < 3
        isDegrees = false; % Default to radians
    end

    % Compute raw difference
    rawDiff = angle1 - angle2;

    if isDegrees
        % Normalize to [-180, 180] for degrees
        diff = mod(rawDiff + 180, 360) - 180;
    else
        % Normalize to [-pi, pi] for radians
        diff = mod(rawDiff + pi, 2*pi) - pi;
    end
end