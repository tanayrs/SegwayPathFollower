clc;
clear all;
close all;

PLOT_ALL = false;

%% System Initialisation
global p;
p = params();

d = p.d; dw = p.dw; g = p.g; m = p.m; b = p.b; I = p.I;

% States are r, phi, psi, V, phidot, psidot %
A = [0,                  0, 0,    1, 0, 0;
     0,                  0, 0,    0, 1, 0;
     0,                  0, 0,    0, 0, 1;
     0,        (d^2*g*m)/I, 0, -b/m, 0, 0;
     0, -(d*(d*m - g*m))/I, 0,    0, 0, 0;
     0,                  0, 0,    0, 0, 0];

% Inputs are Fr and Fl %
B = [                0,                 0;
                     0,                 0;
                     0,                 0;
     (m*d^2 + I)/(I*m), (m*d^2 + I)/(I*m);
                   d/I,               d/I;
                  dw/I,             -dw/I];

C = eye(6);

D = zeros(size(B));

sys = ss(A,B,C,D);

%% LQR Design %%
Q =  diag([1e1 1e2 1e1 0 0 0]);
R = diag([0.1 0.1]);
K = lqr(sys, Q, R);

%% Initial Conditions %%
z0 = [0; 0; 0; 0.2; 0; 0; 0; 0];
z_ref = [1; 0; deg2rad(120); 0; 0; 0];

tspan = [0 100];
p.time_scale = tspan(end)/10;
% p.time_scale = 1;

%% ODE %%
rhs = @(t,z) myrhs_lqr(z,z_ref,t,K);

options = odeset('AbsTol',1e-6, 'RelTol', 1e-6);
solution = ode45(rhs, tspan, z0, options);

animate(solution, z0, tspan, p, 0)

figure;
plot_errors(solution, z0, tspan, p);