function waypoints =  generate_waypoints()
    % Define waypoints to folow a given trajectory
    waypoints.x = linspace(0,10,1000);
    waypoints.y = sin(waypoints.x);