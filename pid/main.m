clc;
clear all;
close all;

PLOT_ALL = false;

%% System Initialisation
global p;
p = params();

d = p.d; dw = p.dw; g = p.g; m = p.m; b = p.b; I = p.I;

%% Initial Conditions %%
z0 = [0; 0; 0; 0; 0; 0; 0; 0];
z_ref = [0; 0; 0; 0; 0; 0];

tspan = [0 500];
p.time_scale = tspan(end)/10;
% p.time_scale = 1;

%% ODE %%
rhs = @(t,z) myrhs(z,t);

options = odeset('AbsTol',1e-6, 'RelTol', 1e-6);
solution = ode45(rhs, tspan, z0, options);

animate(solution, z0, tspan, p, 0)

plot_errors(solution, z0, tspan, p)
