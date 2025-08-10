function p = params()
    p.m = 0.841; % in kg
    p.d = 0.2; % in m
    p.dw = 0.075; % in m
    p.R_wheel = 0.05; % in m
    p.g = 10; % in m/s^2
    
    p.time_scale = 0.1;
    
    p.rated_torque = 25*p.g/100; % Rated Torque of Pendulum Motor
    
    Ig = 0; % Mass Moment of Inertia of Mass about mass
    p.I = (p.m * (p.d ^ 2)) + Ig; % Mass Moment of Intertia about G
    
    p.b = 0.4; % Viscous Drag

    p.int_psi = 0.0;
    p.int_phi = 0.0;
    p.int_r = 0.0;
    p.prev_dt = tic();

    p.curr_waypoint = 1; % Initialise Waypoint
    p.curr_task = 0; % Initialise Waypoint Task
    x = linspace(0,10,1000);
    p.waypoints = generate_waypoints(x);
end