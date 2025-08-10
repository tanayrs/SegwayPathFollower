clc;
clear all;
close all;

PLOT_ALL = false;

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

[num_Fr, den_Fr] = ss2tf(A,B,C,D,1);
[num_Fl, den_Fl] = ss2tf(A,B,C,D,2);

phi_tf = tf(num_Fr(2,:), den_Fr);

psi_tf_Fr = minreal(tf(num_Fr(3,:), den_Fr));
psi_tf_Fl = minreal(tf(num_Fl(3,:), den_Fl));

sys_lqr = ss(A_new, B_new, C, D);


PLOT = false;
if PLOT == true || PLOT_ALL == true
    figure;
    subplot(3,1,1);
    shg;
    hold on;
    plot(solution.x, solution.y(3,:));
    grid on;
    title('Step Plot for r');
    
    subplot(3,1,2);
    plot(solution.x, solution.y(4,:));
    grid on;
    title('Step Plot for \phi');
    hold off;
    
    subplot(3,1,3);
    plot(solution.x, solution.y(5,:));
    grid on;
    title('Step Plot for \psi');
    hold off;
end

PLOT = false;
if PLOT == true || PLOT_ALL == true
    figure;
    shg;
    sigma(sys)
    title('Open Loop Response')
    grid on;
    hold off;
    
    figure;
    shg;
    hold on;
    step(sys_lqr(1,1))
    grid on;
    title('Step Plot for \phi wrt F_{R}');
    
    figure;
    shg;
    hold on;
    sigma(sys_lqr);
    grid on;
    title('Closed Loop Sigma Plot for System');
    hold off;
end