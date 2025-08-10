function [zref, z, curr_waypoint, curr_task] = mywaypoint_controller(z, waypoints, curr_waypoint, curr_task)
    global p;
    % each waypoint has two stages, first fixing heading, then moving to r.
    
    x = z(1);
    y = z(2);
    r = z(3);
    phi = z(4);
    psi = z(5);
    V = z(6);
    phidot = z(7);
    psidot = z(8);

    x_error = waypoints.x(curr_waypoint) - x;
    y_error = waypoints.y(curr_waypoint) - y;

    psi_ref = atan2(y_error,x_error);
    r_error = sqrt(x_error^2 + y_error^2);
    psi_error = psi_ref - psi;

    if curr_task == 0
        zref = [r; 0;psi_ref; 0; 0; 0];
        if abs(psi_error) < 0.05
            curr_task = 1;
            % z = [x;y;0;phi;0;V;phidot;psidot];
        end
    elseif curr_task == 1
        zref = [r_error+r; 0; psi; 0; 0; 0];
        if abs(r_error) < 0.1
            curr_task = 0;
            curr_waypoint = curr_waypoint + 1;
            % z = [x;y;0;phi;0;V;phidot;psidot];
        end
    end
end
