function waypoints = generate_waypoints(x)
    % Define waypoints to follow a triangle wave trajectory
    waypoints.x = x; % X values from 0 to 10m
    waypoints.y = [];
    
    % Sin Wave %
    % for i = 1:length(x)
    %     waypoints.y(end+1) = sin(waypoints.x(i));
    % end

    % Triangle wave parameters
    amplitude = 2;    % Height of the triangle wave
    period = 4;       % Period of the triangle wave

    for i = 1:length(waypoints.x)
        % Calculate position within one period (0 to period)
        x_mod = mod(waypoints.x(i), period);

        % First half of period: rising
        if x_mod <= period/2
            waypoints.y(i) = (2*amplitude/period) * x_mod;
        % Second half of period: falling
        else
            waypoints.y(i) = 2*amplitude - (2*amplitude/period) * x_mod;
        end
    end
end