function d_state = Planar_Space_Robot_Dynamics(t, state, tau, robot_dynamics_constants)
    normalized_theta0 = mod(state(3), 2*pi);
    normalized_theta1 = mod(state(4), 2*pi);
    normalized_theta2 = mod(state(5), 2*pi);
    q = state(1:5);
    q(3) = normalized_theta0;
    q(4) = normalized_theta1;
    q(5) = normalized_theta2;
    q_dot = state(6:10);
    
    [inertial_term_matrix, coriolis_term] = Get_Planar_Dynamic_Matrix(q, q_dot, robot_dynamics_constants);
    %Dq = tau - coriolis_term;
    Dq = tau - coriolis_term;

    q_ddot = inertial_term_matrix \ Dq;

    d_state = [q_dot; q_ddot];
end
