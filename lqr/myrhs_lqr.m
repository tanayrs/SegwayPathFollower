function zdot = myrhs_lqr(z,z_ref,t,K)
    global p;

    x = z(1);
    y = z(2);
    r = z(3);
    phi = z(4);
    psi = z(5);
    V = z(6);
    phidot = z(7);
    psidot = z(8);

    m = p.m; I = p.I; b = p.b; g = p.g; d = p.d; dw = p.dw;

    zsub = [r; phi; psi; V; phidot; psidot];
    % z_ref = [0;0;01;0;0;0];
    
    [z_ref, z, curr_waypoint, curr_task] = mywaypoint_controller(z, p.waypoints, p.curr_waypoint, p.curr_task);

    p.curr_waypoint = curr_waypoint;
    p.curr_task = curr_task;

    if mod(curr_waypoint,50) == 0
        curr_waypoint;
    end

    if (curr_waypoint > length(p.waypoints.x))
        p.curr_waypoint = length(p.waypoints.x);
        % We need to do sth when we reach the last waypoint to avoid errors
    end

    F = K*(z_ref - zsub);
    Fr = F(1);
    Fl = F(2);

    Vdot = (Fl*I + Fr*I - I*V*b + Fl*d^2*m + Fr*d^2*m - V*b*d^2*m + ...
        (d^2*g*m^2*sin(2*phi))/2 - d^3*m^2*phidot^2*sin(phi) - ...
        I*d*m*phidot^2*sin(phi) + V*b*d^2*m*cos(phi)^2)/(m*(I + d^2*m - ...
        d^2*m*cos(phi)^2));

    phiddot = (d*(- d*m*cos(phi)*sin(phi)*phidot^2 + Fl*cos(phi) + ...
        Fr*cos(phi) + g*m*sin(phi)))/(I + d^2*m*sin(phi)^2);

    psiddot = -(dw*(Fl - Fr))/I;

    zdot(1) = V*cos(psi);
    zdot(2) = V*sin(psi);
    zdot(3) = V;
    zdot(4) = phidot;
    zdot(5) = psidot;
    zdot(6) = Vdot;
    zdot(7) = phiddot;
    zdot(8) = psiddot;

    zdot = zdot';
end
