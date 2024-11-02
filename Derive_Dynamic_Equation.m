clc;clear;

%% Dynamic equation derivation
syms r0x(t) r0y(t) theta0(t) theta1(t) theta2(t) b_0 l_1 l_2 m_0 m_1 m_2 I_c0 I_c1 I_c2 t

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

%% State vectors

xc0 = [r0x; r0y];
xc1 = [r0x + b_0*c0 + l_1 * c01 / 2; r0y + b_0*s0 + l_1 * s01 / 2];
xc2 = [r0x + b_0*c0 + l_1 * c01 + l_2 * c012 / 2; r0y + b_0*s0 + l_1*s01 + l_2*s012/2];
Theta0 = theta0;
Theta1 = theta0 + theta1;
Theta2 = theta0 + theta1 + theta2;

vc0 = diff(xc0, t, 1);
vc1 = diff(xc1, t, 1);
vc2 = diff(xc2, t, 1);
omega0 = diff(Theta0, t, 1);
omega1 = diff(Theta1, t, 1);
omega2 = diff(Theta2, t, 1);

%% Kinetic and potential energy
Ek0 = 0.5 * m_0 * transpose(vc0) * vc0 + 0.5 * I_c0 * omega0^2;
Ek1 = 0.5 * m_1 * transpose(vc1) * vc1 + 0.5 * I_c1 * omega1^2;
Ek2 = 0.5 * m_2 * transpose(vc2) * vc2 + 0.5 * I_c2 * omega2^2;

%% Lagrangian
L = Ek0 + Ek1 + Ek2;

%% Lagrangian mechanics derivation 
clc
function result = euler_lagrange_dif(L, state, t)
    state_dot = diff(state, t, 1);
    dLds_dot = diff(L, state_dot, 1);
    ddLds_dot = diff(dLds_dot, t, 1);
    dLds = diff(L, state, 1);
    result = ddLds_dot - dLds;
end

% Each row
f_0x = euler_lagrange_dif(L, r0x(t), t);
f_0y = euler_lagrange_dif(L, r0y(t), t);
tau_0 = euler_lagrange_dif(L, theta0(t), t);
tau_1 = euler_lagrange_dif(L, theta1(t), t);
tau_2 = euler_lagrange_dif(L, theta2(t), t);

%% Selector of each term:
clc
function result = d_selector(equation, variable, t)
    variable_ddot = diff(variable, t, 2);
    result = diff(equation, variable_ddot, 1);
end

function equation = equation_simplifier(equation, t)
    syms c0 c1 c2 c01 c02 c12 c012 s0 s1 s2 s01 s02 s12 s012 theta0(t) theta1(t) theta2(t)
    equation = subs(equation, cos(theta0(t)), c0);
    equation = subs(equation, cos(theta1(t)), c1);
    equation = subs(equation, cos(theta2(t)), c2);
    equation = subs(equation, cos(theta0(t) + theta1(t)), c01);
    equation = subs(equation, cos(theta0(t) + theta2(t)), c02);
    equation = subs(equation, cos(theta1(t) + theta2(t)), c12);
    equation = subs(equation, cos(theta0(t) + theta1(t) + theta2(t)), c012);
    equation = subs(equation, sin(theta0(t)), s0);
    equation = subs(equation, sin(theta1(t)), s1);
    equation = subs(equation, sin(theta2(t)), s2);
    equation = subs(equation, sin(theta0(t) + theta1(t)), s01);
    equation = subs(equation, sin(theta0(t) + theta2(t)), s02);
    equation = subs(equation, sin(theta1(t) + theta2(t)), s12);
    equation = subs(equation, sin(theta0(t) + theta1(t) + theta2(t)), s012);
end

% f_0x, f_0y, tau_0, tau_1, tau_2
% r0x(t), r0y(t), theta0(t), theta1(t), theta2(t)]
% Use this function to yield each h element
%equation_simplifier(d_selector(tau_2, theta2(t), t), t)
h11 = equation_simplifier(d_selector(f_0x, r0x(t), t), t)
h12 = equation_simplifier(d_selector(f_0x, r0y(t), t), t)
h13 = equation_simplifier(d_selector(f_0x, theta0(t), t), t)
h14 = equation_simplifier(d_selector(f_0x, theta1(t), t), t)
h15 = equation_simplifier(d_selector(f_0x, theta2(t), t), t)

h21 = equation_simplifier(d_selector(f_0y, r0x(t), t), t)
h22 = equation_simplifier(d_selector(f_0y, r0y(t), t), t)
h23 = equation_simplifier(d_selector(f_0y, theta0(t), t), t)
h24 = equation_simplifier(d_selector(f_0y, theta1(t), t), t)
h25 = equation_simplifier(d_selector(f_0y, theta2(t), t), t)

h31 = equation_simplifier(d_selector(tau_0, r0x(t), t), t)
h32 = equation_simplifier(d_selector(tau_0, r0y(t), t), t)
h33 = equation_simplifier(d_selector(tau_0, theta0(t), t), t)
h34 = equation_simplifier(d_selector(tau_0, theta1(t), t), t)
h35 = equation_simplifier(d_selector(tau_0, theta2(t), t), t)

h41 = equation_simplifier(d_selector(tau_1, r0x(t), t), t);
h42 = equation_simplifier(d_selector(tau_1, r0y(t), t), t);
h43 = equation_simplifier(d_selector(tau_1, theta0(t), t), t);
h44 = equation_simplifier(d_selector(tau_1, theta1(t), t), t);
h45 = equation_simplifier(d_selector(tau_1, theta2(t), t), t);

h51 = equation_simplifier(d_selector(tau_2, r0x(t), t), t);
h52 = equation_simplifier(d_selector(tau_2, r0y(t), t), t);
h53 = equation_simplifier(d_selector(tau_2, theta0(t), t), t);
h54 = equation_simplifier(d_selector(tau_2, theta1(t), t), t);
h55 = equation_simplifier(d_selector(tau_2, theta2(t), t), t);

H = [h11,h12,h13,h14,h15; h21,h22,h23,h24,h25; h31,h32,h33,h34,h35; h41,h42,h43,h44,h45; h51,h52,h53,h54,h55]

%% Get f
% f_0x, f_0y, tau_0, tau_1, tau_2
% r0x(t), r0y(t), theta0(t), theta1(t), theta2(t)]

Dq_dd_1 = h11 * diff(r0x(t), t, 2) + h12 * diff(r0y(t), t, 2) + h13 * diff(theta0(t), t, 2) + h14 * diff(theta1(t), t, 2) + h15 * diff(theta2(t), t, 2);
Dq_dd_2 = h21 * diff(r0x(t), t, 2) + h22 * diff(r0y(t), t, 2) + h23 * diff(theta0(t), t, 2) + h24 * diff(theta1(t), t, 2) + h25 * diff(theta2(t), t, 2);
Dq_dd_3 = h31 * diff(r0x(t), t, 2) + h32 * diff(r0y(t), t, 2) + h33 * diff(theta0(t), t, 2) + h34 * diff(theta1(t), t, 2) + h35 * diff(theta2(t), t, 2);
Dq_dd_4 = h41 * diff(r0x(t), t, 2) + h42 * diff(r0y(t), t, 2) + h43 * diff(theta0(t), t, 2) + h44 * diff(theta1(t), t, 2) + h45 * diff(theta2(t), t, 2);
Dq_dd_5 = h51 * diff(r0x(t), t, 2) + h52 * diff(r0y(t), t, 2) + h53 * diff(theta0(t), t, 2) + h54 * diff(theta1(t), t, 2) + h55 * diff(theta2(t), t, 2);

% f_0x = euler_lagrange_dif(L, r0x(t), t);
% f_0y = euler_lagrange_dif(L, r0y(t), t);
% tau_0 = euler_lagrange_dif(L, theta0(t), t);
% tau_1 = euler_lagrange_dif(L, theta1(t), t);
% tau_2 = euler_lagrange_dif(L, theta2(t), t);
% Get f
f1 = f_0x - Dq_dd_1;
f2 = f_0y - Dq_dd_2;
f3 = tau_0 - Dq_dd_3;
f4 = tau_1 - Dq_dd_4;
f5 = tau_2 - Dq_dd_5;

%% Simplify f
f1 = simplify(f1);
f2 = simplify(f2);
f3 = simplify(f3);
f4 = simplify(f4);
f5 = simplify(f5);

%% Rewrite into directly copyable form
function equation = equation_simplifier_trig(equation, t)
    syms c0 c1 c2 c01 c02 c12 c012 s0 s1 s2 s01 s02 s12 s012 theta0(t) theta1(t) theta2(t)
    equation = subs(equation, cos(theta0(t)), c0);
    equation = subs(equation, cos(theta1(t)), c1);
    equation = subs(equation, cos(theta2(t)), c2);
    equation = subs(equation, cos(theta0(t) + theta1(t)), c01);
    equation = subs(equation, cos(theta0(t) + theta2(t)), c02);
    equation = subs(equation, cos(theta1(t) + theta2(t)), c12);
    equation = subs(equation, cos(theta0(t) + theta1(t) + theta2(t)), c012);
    equation = subs(equation, sin(theta0(t)), s0);
    equation = subs(equation, sin(theta1(t)), s1);
    equation = subs(equation, sin(theta2(t)), s2);
    equation = subs(equation, sin(theta0(t) + theta1(t)), s01);
    equation = subs(equation, sin(theta0(t) + theta2(t)), s02);
    equation = subs(equation, sin(theta1(t) + theta2(t)), s12);
    equation = subs(equation, sin(theta0(t) + theta1(t) + theta2(t)), s012);
end

function equation = equation_simplifier_derivatives(equation, t)
    syms theta0(t) theta1(t) theta2(t) theta0_dot theta1_dot theta2_dot
    equation = subs(equation, diff(theta0(t), t), theta0_dot);
    equation = subs(equation, diff(theta1(t), t), theta1_dot);
    equation = subs(equation, diff(theta2(t), t), theta2_dot);
end

f1 = equation_simplifier_trig(f1, t);
f2 = equation_simplifier_trig(f2, t);
f3 = equation_simplifier_trig(f3, t);
f4 = equation_simplifier_trig(f4, t);
f5 = equation_simplifier_trig(f5, t);

f1 = equation_simplifier_derivatives(f1, t);
f2 = equation_simplifier_derivatives(f2, t);
f3 = equation_simplifier_derivatives(f3, t);
f4 = equation_simplifier_derivatives(f4, t);
f5 = equation_simplifier_derivatives(f5, t);

%%
f1 = simplify(f1);
f2 = simplify(f2);
f3 = simplify(f3);
f4 = simplify(f4);
f5 = simplify(f5);

disp([f1;f2;f3;f4;f5])

%% Verify
suppose_to_be_0 = f1 + Dq_dd_1 - f_0x
sub(suppose_to_be_0, )
latex(suppose_to_be_0)