%% This file derives the dynamics of the Segway.
%   - zdot contains the full non-linear dynamics.
%   - A_num, B_num contains the linearized state space representation %%

clc;
clear all;

% States %
syms x y r psi phi V omega phidot psidot real;

% Derivatives %
syms xdot ydot Vdot omegadot phiddot psiddot real;

% Parameters %
syms M m b I l g d dw real;

% Inputs %
syms Fr Fl real;

% Define unit vectors
i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

% Define position-dependent vectors
lambda = [cos(psi); sin(psi); 0];
nhat = [-sin(psi); cos(psi); 0];

er = [-sin(phi)*cos(psi); -sin(phi)*sin(psi); cos(phi)];
et = [-cos(phi)*cos(psi); -cos(phi)*sin(psi); -sin(phi)];

% Relative acceleration
arel = (Vdot*lambda + et*d*phiddot - er*d*phidot^2);

% Linear momentum derivative for balance
Ldot_tot_rhs = m*(Vdot*lambda - (d*(phidot^2)*er) + (d*phiddot*et));
Ldot_tot_lhs = (Fr+Fl)*lambda - b*V*lambda;

% Angular momentum derivative for balance
Hdot_pend_rhs = cross(d*er,m*arel) + (-phiddot*I*nhat);
Hdot_pend_lhs = cross(d*er,-m*g*k) + cross(d*er,-b*V*lambda);

% Angular momentum derivative for turning
Hdot_turn_rhs = I*psiddot*k;
Hdot_turn_lhs = cross(dw*nhat,Fl*lambda)+cross(-dw*nhat,Fr*lambda);

% Equations of motion
eqn1 = dot(Ldot_tot_rhs - Ldot_tot_lhs,lambda);
eqn2 = dot(Hdot_pend_rhs - Hdot_pend_lhs,-nhat);
eqn3 = dot(Hdot_turn_rhs - Hdot_turn_lhs, k);

% Create equations in standard form (moved F and M terms)
eq1 = collect(eqn1, [Vdot, phiddot]);
eq2 = collect(eqn2, [Vdot, phiddot]);
eq3 = collect(eqn3, psiddot);

[Vdot, phiddot] = solve([eq1, eq2], [Vdot, phiddot]);
psiddot = solve(eq3, psiddot);

Vdot = simplify(Vdot);
phiddot = simplify(phiddot);
psiddot = simplify(psiddot);

rdot = V;

% Vdot = (Fl*I + Fr*I - I*V*b + Fl*d^2*m + Fr*d^2*m - V*b*d^2*m + (d^2*g*(m^2)*2*phi)/2 + V*b*d^2*m)/(m*(I + d^2*m - d^2*m));
% phiddot = (d*(- d*m*phi + Fl + Fr + g*m*phi))/I;

% State equations (after solving for Vdot and thetaddot)
zdot = [xdot;
        ydot;
        rdot;
        phidot;
        psidot;
        Vdot;
        phiddot;
        psiddot]

% Define state and input vectors
z = [x; y; r; phi; psi; V; phidot; psidot];
u = [Fr; Fl];

% Calculate Jacobians for linearization around equilibrium point
% Note: Need to substitute the solved Vdot and thetaddot first
A = jacobian(zdot, z)
B = jacobian(zdot, u)


% Equilibrium point (upward position)
% z_eq = [0; 0; 0; 0];
% u_eq = [0];

% Substitute equilibrium point
% A_linear = subs(A, [z', u'], [z_eq', u_eq'])
% B_linear = subs(B, [z', u'], [z_eq', u_eq'])

% To get numerical matrices:
% A_num = double(subs(A_linear, {g, m, M, d, I, b}, {9.81, m_val, M_val, d_val, I_val, b_val}));
% B_num = double(subs(B_linear, {g, m, M, d, I, b}, {9.81, m_val, M_val, d_val, I_val, b_val}));