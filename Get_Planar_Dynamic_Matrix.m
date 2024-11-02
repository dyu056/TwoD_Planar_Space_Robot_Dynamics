% Man fuck finally it matches hahahahahahahaha

function [H, F] = Get_Planar_Dynamic_Matrix(state_vector, state_dot, robot_dynamics_constants)
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

    m_0 = robot_dynamics_constants.m0;
    m_1 = robot_dynamics_constants.m1;
    m_2 = robot_dynamics_constants.m2;
    b_0 = robot_dynamics_constants.b0;
    l_1 = robot_dynamics_constants.l1;
    l_2 = robot_dynamics_constants.l2;
    I_c0 = robot_dynamics_constants.Ic0;
    I_c1 = robot_dynamics_constants.Ic1;
    I_c2 = robot_dynamics_constants.Ic2;



    % Define the h matrix elements based on the equations provided
    h11 = m_0 + m_1 + m_2;
    h12 = 0;
    h13 = - (m_1*(2*b_0*s0 + l_1*s01))/2 - (m_2*(2*b_0*s0 + 2*l_1*s01 + l_2*s012))/2;
    h14 = - (m_2*(2*l_1*s01 + l_2*s012))/2 - (l_1*m_1*s01)/2;
    h15 = -(l_2*m_2*s012)/2;
    
    h21 = 0;
    h22 = m_0 + m_1 + m_2;
    h23 = (m_1*(2*b_0*c0 + c01*l_1))/2 + (m_2*(2*b_0*c0 + 2*c01*l_1 + c012*l_2))/2;
    h24 = (m_2*(2*c01*l_1 + c012*l_2))/2 + (c01*l_1*m_1)/2;
    h25 = (c012*l_2*m_2)/2;
    
    h31 = - m_1*(b_0*s0 + (l_1*s01)/2) - m_2*(b_0*s0 + l_1*s01 + (l_2*s012)/2);
    h32 = m_1*(b_0*c0 + (c01*l_1)/2) + m_2*(b_0*c0 + c01*l_1 + (c012*l_2)/2);
    h33 = I_c0 + I_c1 + I_c2 + m_2*(b_0*c0 + c01*l_1 + (c012*l_2)/2)^2 + m_2*(b_0*s0 + l_1*s01 + (l_2*s012)/2)^2 + m_1*(b_0*c0 + (c01*l_1)/2)^2 + m_1*(b_0*s0 + (l_1*s01)/2)^2;
    h34 = I_c1 + I_c2 + m_2*(c01*l_1 + (c012*l_2)/2)*(b_0*c0 + c01*l_1 + (c012*l_2)/2) + m_2*(l_1*s01 + (l_2*s012)/2)*(b_0*s0 + l_1*s01 + (l_2*s012)/2) + (c01*l_1*m_1*(b_0*c0 + (c01*l_1)/2))/2 + (l_1*m_1*s01*(b_0*s0 + (l_1*s01)/2))/2;
    h35 = I_c2 + (c012*l_2*m_2*(b_0*c0 + c01*l_1 + (c012*l_2)/2))/2 + (l_2*m_2*s012*(b_0*s0 + l_1*s01 + (l_2*s012)/2))/2;
    
    h41 = - m_2*(l_1*s01 + (l_2*s012)/2) - (l_1*m_1*s01)/2;
    h42 = m_2*(c01*l_1 + (c012*l_2)/2) + (c01*l_1*m_1)/2;
    h43 = I_c1 + I_c2 + m_2*(c01*l_1 + (c012*l_2)/2)*(b_0*c0 + c01*l_1 + (c012*l_2)/2) + m_2*(l_1*s01 + (l_2*s012)/2)*(b_0*s0 + l_1*s01 + (l_2*s012)/2) + (c01*l_1*m_1*(b_0*c0 + (c01*l_1)/2))/2 + (l_1*m_1*s01*(b_0*s0 + (l_1*s01)/2))/2;
    h44 = I_c1 + I_c2 + m_2*(l_1*s01 + (l_2*s012)/2)^2 + m_2*(c01*l_1 + (c012*l_2)/2)^2 + (c01^2*l_1^2*m_1)/4 + (l_1^2*m_1*s01^2)/4;
    h45 = I_c2 + (c012*l_2*m_2*(c01*l_1 + (c012*l_2)/2))/2 + (l_2*m_2*s012*(l_1*s01 + (l_2*s012)/2))/2;
    
    h51 = -(l_2*m_2*s012)/2;
    h52 = (c012*l_2*m_2)/2;
    h53 = I_c2 + (c012*l_2*m_2*(b_0*c0 + c01*l_1 + (c012*l_2)/2))/2 + (l_2*m_2*s012*(b_0*s0 + l_1*s01 + (l_2*s012)/2))/2;
    h54 = I_c2 + (c012*l_2*m_2*(c01*l_1 + (c012*l_2)/2))/2 + (l_2*m_2*s012*(l_1*s01 + (l_2*s012)/2))/2;
    h55 = (m_2*c012^2*l_2^2)/4 + (m_2*l_2^2*s012^2)/4 + I_c2;
    
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
    % F = [f1; f2; f3; f4; f5];
end
