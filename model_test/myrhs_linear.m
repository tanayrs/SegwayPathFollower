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

    A = [0,                  0, 0,    1, 0, 0;
         0,                  0, 0,    0, 1, 0;
         0,                  0, 0,    0, 0, 1;
         0,        (d^2*g*m)/I, 0, -b/m, 0, 0;
         0, -(d*(d*m - g*m))/I, 0,    0, 0, 0;
         0,                  0, 0,    0, 0, 0];

    B = [                0,                 0;
                     0,                 0;
                     0,                 0;
     (m*d^2 + I)/(I*m), (m*d^2 + I)/(I*m);
                   d/I,               d/I;
                  dw/I,             -dw/I];

    z_sub = [r; phi; psi; V; phidot; psidot];
    u = [Fr; Fl];
    w = (A*z_sub);
    w2 = (B*u);
    w = w+w2;

    zdot(1) = V*cos(psi);
    zdot(2) = V*sin(psi);
    zdot(3) = w(1);
    zdot(4) = w(2);
    zdot(5) = w(3);
    zdot(6) = w(4);
    zdot(7) = w(5);
    zdot(8) = w(6);

    zdot = zdot';
end