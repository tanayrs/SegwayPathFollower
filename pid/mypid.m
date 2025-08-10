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

%% Controller Design: Steering Controller %%

Kp_psi_Fl = -40;
Ki_psi_Fl = 0;
Kd_psi_Fl = -5;

Kp_psi_Fr = 40;
Ki_psi_Fr = 0;
Kd_psi_Fr = 5;

controller_psi_Fl = tf([Kd_psi_Fl, Kp_psi_Fl, Ki_psi_Fl], [1 0]);
controller_psi_Fr = tf([Kd_psi_Fr, Kp_psi_Fr, Ki_psi_Fr], [1 0]);

cltf_psi_Fl = feedback(controller_psi_Fl*psi_tf_Fl,1);
cltf_psi_Fr = feedback(controller_psi_Fr*psi_tf_Fr,1);

PLOT = false;
if PLOT == true || PLOT_ALL == true
    figure;
    step(cltf_psi_Fl);
    title("CTLF Step Response \psi wrt F_{L}");
    
    figure;
    step(cltf_psi_Fr);
    title("CTLF Step Response \psi wrt F_{R}");
end

%% Controller Design: Tilt Controller %%

Kp_phi = 20;
Ki_phi = 0;
Kd_phi = 1;

controller_phi = tf([Kd_phi, Kp_phi, Ki_phi], [1 0]);

cltf_phi = feedback(controller_phi * phi_tf,1);

PLOT = false;
if PLOT == true || PLOT_ALL == true
    % step(cltf_phi)
    lsim(cltf_phi, 0.3*ones(size(0:0.01:3)), 0:0.01:3)
end

%% Controller Deisgn: R Controller %%
Kp_R = -0.01;
Ki_R = 0;
Kd_R = 0;

controller_R = tf([Kd_R, Kp_R, Ki_R], [1 0]);

cltf_R = feedback(cltf_phi*controller_R,1);

PLOT = false;
if PLOT == true || PLOT_ALL == true
    figure;
    % step(cltf_R)
    lsim(cltf_R, 0.3*ones(size(0:0.01:3)), 0:0.01:3)
    title("Step Response CLTF Position in \hat{\lambda}")
end