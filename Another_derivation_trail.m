%%
clc;clear;

%% Dynamic equation derivation
syms r0x r0y theta0 theta1 theta2
syms dr0x dr0y dtheta0 dtheta1 dtheta2
syms f_0x f_0y tau_0 tau_1 tau_2 
syms b_0 l_1 l_2 m_0 m_1 m_2 I_c0 I_c1 I_c2

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

% State vectors

xc0 = [r0x; r0y];
xc1 = [r0x + b_0*c0 + l_1 * c01 / 2; r0y + b_0*s0 + l_1 * s01 / 2];
xc2 = [r0x + b_0*c0 + l_1 * c01 + l_2 * c012 / 2; r0y + b_0*s0 + l_1*s01 + l_2*s012/2];

vc0 = [dr0x; dr0y];
vc1 = [dr0x - b_0*s0*dtheta0 - l_1*s01*(dtheta0+dtheta1)/2; dr0y + b_0*c0*dtheta0 + l_1*c01*(dtheta0+dtheta1)/2];
vc2 = [dr0x - b_0*s0*dtheta0 - l_1*s01*(dtheta0+dtheta1) - (l_2/2)*s012*(dtheta0+dtheta1+dtheta2); dr0y+b_0*c0*dtheta0+l_1*c01*(dtheta0+dtheta1)+(l_2/2)*c012*(dtheta0+dtheta1+dtheta2)];
omega0 = dtheta0;
omega1 = dtheta0 + dtheta1;
omega2 = dtheta0 + dtheta1 + dtheta2;

% Kinetic and potential energy
Ek0 = 0.5 * m_0 * transpose(vc0) * vc0 + 0.5 * I_c0 * omega0^2;
Ek1 = 0.5 * m_1 * transpose(vc1) * vc1 + 0.5 * I_c1 * omega1^2;
Ek2 = 0.5 * m_2 * transpose(vc2) * vc2 + 0.5 * I_c2 * omega2^2;

%% Lagrangian calc
L = Ek0 + Ek1 + Ek2;
X   = {r0x dr0x r0y dr0y theta0 dtheta0 theta1 dtheta1 theta2 dtheta2};
Q_i = {0 0 0 0 0};
Q_e = {f_0x f_0y tau_0 tau_1 tau_2};
R   = 0;
par = {b_0 l_1 l_2 m_0 m_1 m_2 I_c0 I_c1 I_c2};

%% Start popping
% VF  = EulerLagrange(L,X,Q_i,Q_e,R,par,'m','s','2D space robot');
VF  = EulerLagrange(L,X,Q_i,Q_e,R,par,'m','s');

 