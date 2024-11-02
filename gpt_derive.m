clc; clear;

%% Dynamic equation derivation
syms r0x(t) r0y(t) theta0(t) theta1(t) theta2(t) b_0 l_1 l_2 m_0 m_1 m_2 I_c0 I_c1 I_c2 t
syms dr0x_sym(t) dr0y_sym(t) dtheta0_sym(t) dtheta1_sym(t) dtheta2_sym(t)  % symbolic functions for velocities

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

%% State vectors (explicit time dependency for each component)
xc0 = [r0x; r0y];
xc1 = [r0x + b_0*c0 + l_1 * c01 / 2; r0y + b_0*s0 + l_1 * s01 / 2];
xc2 = [r0x + b_0*c0 + l_1 * c01 + l_2 * c012 / 2; r0y + b_0*s0 + l_1*s01 + l_2*s012/2];
Theta0 = theta0;
Theta1 = theta0 + theta1;
Theta2 = theta0 + theta1 + theta2;

% Compute velocities by differentiating each component explicitly
vc0 = diff(xc0, t);
vc1 = diff(xc1, t);
vc2 = diff(xc2, t);
omega0 = diff(Theta0, t);
omega1 = diff(Theta1, t);
omega2 = diff(Theta2, t);

%% Kinetic and potential energy
Ek0 = 0.5 * m_0 * transpose(vc0) * vc0 + 0.5 * I_c0 * omega0^2;
Ek1 = 0.5 * m_1 * transpose(vc1) * vc1 + 0.5 * I_c1 * omega1^2;
Ek2 = 0.5 * m_2 * transpose(vc2) * vc2 + 0.5 * I_c2 * omega2^2;

%% Lagrangian
L = Ek0 + Ek1 + Ek2;

%% Substitute velocities with placeholders in Lagrangian
L = subs(L, [diff(r0x, t), diff(r0y, t), diff(theta0, t), diff(theta1, t), diff(theta2, t)], ...
             [dr0x_sym(t), dr0y_sym(t), dtheta0_sym(t), dtheta1_sym(t), dtheta2_sym(t)]);

%% Generalized coordinates and velocities
q = [r0x; r0y; theta0; theta1; theta2];           % Generalized coordinates
dq = [dr0x_sym(t); dr0y_sym(t); dtheta0_sym(t); dtheta1_sym(t); dtheta2_sym(t)]; % Symbolic functions for generalized velocities

%% Inertia Matrix (Mass Matrix)
D = sym(zeros(5)); % Initialize the inertia matrix
for i = 1:5
    for j = 1:5
        % Compute second derivatives with respect to placeholder velocity variables
        vi = dq(i);
        vj = dq(j);
        
        dL_dqdq = functionalDerivative(functionalDerivative(L, vi), vj); % ∂²L/∂(dq_i)∂(dq_j)
        D(i, j) = simplify(dL_dqdq);
    end
end
disp('Inertia Matrix (D):');
disp(D);

%% Coriolis Matrix
C = sym(zeros(5)); % Initialize the Coriolis matrix
for k = 1:5
    for j = 1:5
        c_kj = 0; % Initialize each Coriolis term
        for i = 1:5
            % Compute Christoffel symbols of the first kind with placeholders
            c_ijk = 0.5 * (diff(D(k, j), q(i)) + ...
                           diff(D(k, i), q(j)) - ...
                           diff(D(i, j), q(k))) * dq(i);
            c_kj = c_kj + c_ijk; % Sum up the terms for each entry
        end
        C(k, j) = simplify(c_kj);
    end
end
disp('Coriolis Matrix (C):');
disp(C);