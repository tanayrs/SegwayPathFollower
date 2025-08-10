function [Fr, Fl] = controller(z, zref)
    global p;

    x = z(1);
    y = z(2);
    r = z(3);
    phi = z(4);
    psi = z(5);
    V = z(6);
    phidot = z(7);
    psidot = z(8);

    r_ref = zref(3);
    psi_ref = zref(5);

    t_curr = toc();
    dt = double(t_curr - p.prev_dt);
    p.prev_dt = double(t_curr); 

    % Psi Controller %
    Kp_psi = 40;
    Ki_psi = 0;
    Kd_psi = 5;
    
    psi_error = psi_ref - psi;
    p.int_psi = p.int_psi + (psi_error*dt);

    Fr = (Kd_psi * -psidot) +  (Kp_psi * psi_error) + (Ki_psi * p.int_psi);
    Fl = -Fr;

    % R Controller %
    Kp_R = -0.01;
    Ki_R = 0;
    Kd_R = 0;

    r_error = double(r_ref - r);
    r_error_dt = double(r_error) * double(dt);
    p.int_r = double(p.int_r) + r_error_dt;

    phi_ref = (Kd_R * -V) +  (Kp_R * r_error) + (Ki_R * p.int_r);

    if phi_ref > 0.05
        phi_ref = 0.05;
    elseif phi_ref < -0.05
        phi_ref = -0.05;
    end

    % phi_ref = 0;
    
    % Phi Controller %
    Kp_phi = 20;
    Ki_phi = 0; % 25.9557;
    Kd_phi = 1; % 10.092;

    phi_error = phi_ref - phi;
    p.int_phi = p.int_phi + (phi*dt);

    Fr = Fr + (Kd_phi * -phidot) +  (Kp_phi * phi_error) + (Ki_phi * p.int_phi);
    Fl = Fl + (Kd_phi * -phidot) +  (Kp_phi * phi_error) + (Ki_phi * p.int_phi);
end