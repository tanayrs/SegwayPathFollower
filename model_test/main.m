clc;
clear all;
close all;

%% Defining Parameter Struct
% Define Parameters
p = params();

move_frame = 0;
%% Define Time Span 
tend = 1; tspan = [0, tend];

% Define Initial Conditions
x0 = [0; 0; 0]; angles0 = [deg2rad(1); 0]; rates0 = [0; 0; 0]; 
z0 = [x0; angles0; rates0];

%% Get Function
rhs = @(t, z) myrhs(t, z, p);
rhs_linear = @(t,z) myrhs_linear(t,z,p);

options = odeset('AbsTol', 1e-6, 'RelTol', 1e-6);
solution = ode45(rhs, tspan, z0, options);
solution_linear = ode45(rhs_linear,tspan,z0,options);

z = solution.y;
theta = z(4,:);
t = solution.x;

z_linear = solution_linear.y;
theta_linear = z_linear(4,:);
t_linear = solution_linear.x;

figure;
plot(t, theta,'DisplayName',"Non-Linear");
hold on;
plot(t_linear, theta_linear,'DisplayName',"Linear");
xlabel("Time (s)")
ylabel("\phi (rad)")
legend()
shg;
hold off;

% animate(solution, z0, tspan,p,move_frame);

% plot_transfer_functions(p);

% check_stability(p)
