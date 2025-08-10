function zdot = myrhs(t,z,p)
    x = z(1);
    y = z(2);
    r = z(3);
    phi = z(4);
    psi = z(5);
    V = z(6);
    phidot = z(7);
    psidot = z(8);

    m = p.m; I = p.I; b = p.b; g = p.g; d = p.d; dw = p.dw; 

    [Fr, Fl] = controller(z);

    Vdot = (Fl*I + Fr*I - I*V*b + Fl*d^2*m + Fr*d^2*m - V*b*d^2*m + (d^2*g*m^2*sin(2*phi))/2 - d^3*m^2*phidot^2*sin(phi) - I*d*m*phidot^2*sin(phi) + V*b*d^2*m*cos(phi)^2)/(m*(I + d^2*m - d^2*m*cos(phi)^2));

    phiddot = (d*(- d*m*cos(phi)*sin(phi)*phidot^2 + Fl*cos(phi) + Fr*cos(phi) + g*m*sin(phi)))/(I + d^2*m*sin(phi)^2);
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