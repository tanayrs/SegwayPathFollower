clc;
clear all;
close all;

PLOT_ALL = true;

%% System Initialisation
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

%% System Characteristic Plots %%

PLOT = false;
if PLOT == true || PLOT_ALL == true
    figure;
    hold on;
    bode(sys(1,1));
    grid on;
    title('Bode Plot for \phi wrt F_{R}');

    % figure;
    bode(sys(2,1));
    grid on;
    title('Bode Plot for \psi wrt F_{R}');

    legend('\phi', '\psi');

    hold off;

    figure;
    subplot(2,1,1)
    rlocus(sys(1,1));
    title('Root Locus for \phi wrt F_{R}')
    subplot(2,1,2)
    rlocus(sys(1,2));
    title('Root Locus for \phi wrt F_{L}')

    figure;
    rlocus(sys(2,1));
    title('Root Locus for \psi wrt F_{R}')
    rlocus(sys(2,2));
    title('Root Locus for \psi wrt F_{L}')
end