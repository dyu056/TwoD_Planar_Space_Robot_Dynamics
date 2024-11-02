function [H, F] = Get_Planar_Dynamic_Matrix_obs(state_vector, state_dot, robot_dynamics_constants)
    % state_vector = [r0x, r0y, theta0, theta1, theta2]
    % state_dot = [r0x_dot, r0y_dot, theta0_dot, theta1_dot, theta2_dot]
    % Planar space robot dynamics
    % angles
    theta0 = state_vector(3);
    theta1 = state_vector(4);
    theta2 = state_vector(5);
    theta0_dot = state_dot(3);
    theta1_dot = state_dot(4);
    theta2_dot = state_dot(5);
    c0 = cos(theta0);
    c1 = cos(theta1);
    c2 = cos(theta2);
    c01 = cos(theta0 + theta1);
    c02 = cos(theta0 + theta2);
    c12 = cos(theta1 + theta2);
    c012 = cos(theta0 + theta1 + theta2);
    s0 = sin(theta0);
    s1 = sin(theta1);
    s2 = sin(theta2);
    s01 = sin(theta0 + theta1);
    s02 = sin(theta0 + theta2);
    s12 = sin(theta1 + theta2);
    s012 = sin(theta0 + theta1 + theta2);

    m0 = robot_dynamics_constants.m0;
    m1 = robot_dynamics_constants.m1;
    m2 = robot_dynamics_constants.m2;
    b0 = robot_dynamics_constants.b0;
    l1 = robot_dynamics_constants.l1;
    l2 = robot_dynamics_constants.l2;
    Ic0 = robot_dynamics_constants.Ic0;
    %Ic1 = robot_dynamics_constants.Ic1;
    %Ic2 = robot_dynamics_constants.Ic2;



    % Define the h matrix elements based on the equations provided
    h11 = m0 + m1 + m2;
    h12 = 0;
    h13 = -(m1 + m2)*b0*s0 - ((1/2)*m1 + m2)*l1*s01 - (1/2)*m2*l2*s012;
    h14 = -((1/2)*m1 + m2)*l1*s01 - (1/2)*m2*l2*s012;
    h15 = -(1/2)*m2*l2*s012;
    
    h21 = h12;
    h22 = m0 + m1 + m2;
    h23 = (m1 + m2)*b0*c0 + ((1/2)*m1 + m2)*l1*c01 + (1/2)*m2*l2*c012;
    h24 = ((1/2)*m1 + m2)*l1*c01 + (1/2)*m2*l2*c012;
    h25 = (1/2)*m2*l2*c012;
    
    h31 = h13;
    h32 = h23;
    h33 = (Ic0 + m1*b0^2 + m2*b0^2) + ((1/3)*m1 + m2)*l1^2 + (1/3)*m2*l2^2 + (m1 + 2*m2)*b0*l1*c1 + m2*b0*l2*c12 + m2*l1*l2*c2;
    h34 = ((1/3)*m1 + m2)*l1^2 + (1/3)*m2*l2^2 + ((1/2)*m1 + m2)*b0*l1*c1 + (1/2)*m2*b0*l2*c12 + m2*l1*l2*c2;
    h35 = (1/3)*m2*l2^2 + (1/2)*m2*b0*l2*c12 + (1/2)*m2*l1*l2*c2;
    
    h41 = h14;
    h42 = h24;
    h43 = h34;
    h44 = ((1/3)*m1 + m2)*l1^2 + (1/3)*m2*l2^2 + m2*l1*l2*c2;
    h45 = (1/3)*m2*l2^2 + (1/2)*m2*l1*l2*c2;
    
    h51 = h15;
    h52 = h25;
    h53 = h35;
    h54 = h45;
    h55 = (1/3)*m2*l2^2;
    
    % Define the f vector elements based on the equations provided
    F = [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     - b_0*c0*m_1*theta0_dot^2 - b_0*c0*m_2*theta0_dot^2 - (c01*l_1*m_1*theta0_dot^2)/2 - (c01*l_1*m_1*theta1_dot^2)/2 - c01*l_1*m_2*theta0_dot^2 - c01*l_1*m_2*theta1_dot^2 - (c012*l_2*m_2*theta0_dot^2)/2 - (c012*l_2*m_2*theta1_dot^2)/2 - (c012*l_2*m_2*theta2_dot^2)/2 - c01*l_1*m_1*theta0_dot*theta1_dot - 2*c01*l_1*m_2*theta0_dot*theta1_dot - c012*l_2*m_2*theta0_dot*theta1_dot - c012*l_2*m_2*theta0_dot*theta2_dot - c012*l_2*m_2*theta1_dot*theta2_dot
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     - b_0*m_1*s0*theta0_dot^2 - b_0*m_2*s0*theta0_dot^2 - (l_1*m_1*s01*theta0_dot^2)/2 - (l_1*m_1*s01*theta1_dot^2)/2 - l_1*m_2*s01*theta0_dot^2 - l_1*m_2*s01*theta1_dot^2 - (l_2*m_2*s012*theta0_dot^2)/2 - (l_2*m_2*s012*theta1_dot^2)/2 - (l_2*m_2*s012*theta2_dot^2)/2 - l_1*m_1*s01*theta0_dot*theta1_dot - 2*l_1*m_2*s01*theta0_dot*theta1_dot - l_2*m_2*s012*theta0_dot*theta1_dot - l_2*m_2*s012*theta0_dot*theta2_dot - l_2*m_2*s012*theta1_dot*theta2_dot
(b_0*c01*l_1*m_1*s0*theta1_dot^2)/2 - (b_0*c0*l_1*m_1*s01*theta1_dot^2)/2 - b_0*c0*l_1*m_2*s01*theta1_dot^2 + b_0*c01*l_1*m_2*s0*theta1_dot^2 - (b_0*c0*l_2*m_2*s012*theta1_dot^2)/2 + (b_0*c012*l_2*m_2*s0*theta1_dot^2)/2 - (b_0*c0*l_2*m_2*s012*theta2_dot^2)/2 + (b_0*c012*l_2*m_2*s0*theta2_dot^2)/2 - (c01*l_1*l_2*m_2*s012*theta2_dot^2)/2 + (c012*l_1*l_2*m_2*s01*theta2_dot^2)/2 - b_0*c0*l_1*m_1*s01*theta0_dot*theta1_dot + b_0*c01*l_1*m_1*s0*theta0_dot*theta1_dot - 2*b_0*c0*l_1*m_2*s01*theta0_dot*theta1_dot + 2*b_0*c01*l_1*m_2*s0*theta0_dot*theta1_dot - b_0*c0*l_2*m_2*s012*theta0_dot*theta1_dot + b_0*c012*l_2*m_2*s0*theta0_dot*theta1_dot - b_0*c0*l_2*m_2*s012*theta0_dot*theta2_dot + b_0*c012*l_2*m_2*s0*theta0_dot*theta2_dot - b_0*c0*l_2*m_2*s012*theta1_dot*theta2_dot + b_0*c012*l_2*m_2*s0*theta1_dot*theta2_dot - c01*l_1*l_2*m_2*s012*theta0_dot*theta2_dot + c012*l_1*l_2*m_2*s01*theta0_dot*theta2_dot - c01*l_1*l_2*m_2*s012*theta1_dot*theta2_dot + c012*l_1*l_2*m_2*s01*theta1_dot*theta2_dot
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     (b_0*l_1*m_1*s1*theta0_dot^2)/2 + b_0*l_1*m_2*s1*theta0_dot^2 + (b_0*l_2*m_2*s12*theta0_dot^2)/2 - (l_1*l_2*m_2*s2*theta2_dot^2)/2 - l_1*l_2*m_2*s2*theta0_dot*theta2_dot - l_1*l_2*m_2*s2*theta1_dot*theta2_dot
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (l_2*m_2*(b_0*s12*theta0_dot^2 + l_1*s2*theta0_dot^2 + l_1*s2*theta1_dot^2 + 2*l_1*s2*theta0_dot*theta1_dot))/2];

    
    % Display h and f matrices
    H = [h11, h12, h13, h14, h15;
         h21, h22, h23, h24, h25;
         h31, h32, h33, h34, h35;
         h41, h42, h43, h44, h45;
         h51, h52, h53, h54, h55];
    %F = [f1; f2; f3; f4; f5];
end
