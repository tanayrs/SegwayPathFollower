function retval = animate(solution, z0, tspan, p, move_frame)
    d = p.d;      % Length of pendulum
    dw = p.dw;    % Distance from center to each wheel (assuming this is a parameter)
    R_wheel = p.R_wheel;
    
    x0 = z0(1);
    y0 = z0(2);
    theta0 = z0(4);
    psi0 = z0(5); % Initial heading angle
    
    % Get solution data
    t_vals = linspace(tspan(1), tspan(2), 1e4);
    z_vals = deval(solution,t_vals);
    
    % Initialize figure
    figure;
    shg;
    hold on;
    grid on;
    view(3); % Set 3D view
    
    % Set dynamic axis limits based on the trajectory
    if move_frame == 0
        x_min = min(z_vals(1, :)) - 0.5;
        x_max = max(z_vals(1, :)) + 0.5;
        y_min = min(z_vals(2, :)) - 0.5;
        y_max = max(z_vals(2, :)) + 0.5;
        
        xlim([x_min, x_max]);
        ylim([y_min, y_max]);
        zlim([-0.5*d, 1.5*d]);

        axis equal;
    elseif move_frame == 42
        
        xlim([-10, 10]);
        ylim([-10, 10]);
        zlim([-5, 5]);
        axis equal;
    else
        axis equal;
        zlim([-0.5*d, 1.5*d]);
    end
    
    % Initialize wheel parameters
    wheel_radius = R_wheel;
    theta_wheel = linspace(0, 2*pi, 50);
    
    % Initialize variables for first frame
    psi = psi0; % Heading angle
    phi = theta0; % Pendulum angle
    
    % Define basis vectors based on initial orientation
    lambda = [cos(psi); sin(psi); 0];
    nhat = [-sin(psi); cos(psi); 0];
    er = [-sin(phi)*cos(psi); -sin(phi)*sin(psi); cos(phi)];
    et = [-cos(phi)*cos(psi); -cos(phi)*sin(psi); -sin(phi)];
    
    % Robot center position
    robot_pos = [x0; y0; 0];
    
    % Calculate wheel positions
    left_wheel_pos = robot_pos + dw * nhat;
    right_wheel_pos = robot_pos - dw * nhat;
    
    % Initialize both wheels
    left_wheel_x = zeros(1, length(theta_wheel));
    left_wheel_y = zeros(1, length(theta_wheel));
    left_wheel_z = zeros(1, length(theta_wheel));
    
    right_wheel_x = zeros(1, length(theta_wheel));
    right_wheel_y = zeros(1, length(theta_wheel));
    right_wheel_z = zeros(1, length(theta_wheel));
    
    % Calculate initial wheel positions
    for i = 1:length(theta_wheel)
        % Left wheel
        wheel_offset = wheel_radius * (cos(theta_wheel(i)) * lambda + sin(theta_wheel(i)) * [0; 0; 1]);
        left_wheel_point = left_wheel_pos + wheel_offset;
        left_wheel_x(i) = left_wheel_point(1);
        left_wheel_y(i) = left_wheel_point(2);
        left_wheel_z(i) = left_wheel_point(3);
        
        % Right wheel
        right_wheel_point = right_wheel_pos + wheel_offset;
        right_wheel_x(i) = right_wheel_point(1);
        right_wheel_y(i) = right_wheel_point(2);
        right_wheel_z(i) = right_wheel_point(3);
    end
    
    % Create pendulum rectangle vertices
    pendulum_width = 2*dw;  % Width spans entire robot
    pendulum_thickness = 0.05 * d;  % Thickness of the pendulum
    
    % Center point of the chassis (directly between wheels)
    chassis_center = robot_pos;
    
    % Pendulum as a rectangle (4 corners) - Now aligned with wheel centers
    % Base corners
    bl = chassis_center - (pendulum_width/2) * nhat - pendulum_thickness/2 * et;  % Bottom left
    br = chassis_center + (pendulum_width/2) * nhat - pendulum_thickness/2 * et;  % Bottom right
    
    % Top corners (d along er direction)
    tl = bl + d * er;  % Top left
    tr = br + d * er;  % Top right
    
    % Initialize 3D plots
    left_wheel = plot3(left_wheel_x, left_wheel_y, left_wheel_z, 'w', 'LineWidth', 2);
    right_wheel = plot3(right_wheel_x, right_wheel_y, right_wheel_z, 'w', 'LineWidth', 2);
    
    % Plot pendulum as a rectangle (using patch)
    pendulum_faces = [1 2 3 4]; % Rectangle face
    pendulum_verts = [bl'; br'; tr'; tl'];
    pendulum_rect = patch('Vertices', pendulum_verts, 'Faces', pendulum_faces, ...
                         'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b', 'LineWidth', 1.5);
    
    % Add connecting line between wheels (chassis)
    % chassis = plot3([left_wheel_pos(1), right_wheel_pos(1)], ...
    %                [left_wheel_pos(2), right_wheel_pos(2)], ...
    %                [left_wheel_pos(3), right_wheel_pos(3)], 'w', 'LineWidth', 2.5);
    
    % Draw axles
    % left_axle = plot3([left_wheel_pos(1), left_wheel_pos(1)], ...
    %                  [left_wheel_pos(2), left_wheel_pos(2)], ...
    %                  [left_wheel_pos(3)-wheel_radius/2, left_wheel_pos(3)+wheel_radius/2], ...
    %                  'w-', 'LineWidth', 1.5);
    % 
    % right_axle = plot3([right_wheel_pos(1), right_wheel_pos(1)], ...
    %                   [right_wheel_pos(2), right_wheel_pos(2)], ...
    %                   [right_wheel_pos(3)-wheel_radius/2, right_wheel_pos(3)+wheel_radius/2], ...
    %                   'w-', 'LineWidth', 1.5);
    
    % Add coordinate axes for reference
    arrow_length = d/2;
    quiver3(0, 0, 0, arrow_length, 0, 0, 'r', 'LineWidth', 1.5);
    quiver3(0, 0, 0, 0, arrow_length, 0, 'g', 'LineWidth', 1.5);
    quiver3(0, 0, 0, 0, 0, arrow_length, 'b', 'LineWidth', 1.5);
    text(arrow_length*1.1, 0, 0, 'X');
    text(0, arrow_length*1.1, 0, 'Y');
    text(0, 0, arrow_length*1.1, 'Z');
    
    % Optional: Add trajectory
    
    robot_traj_x = z_vals(1,:);
    robot_traj_y = z_vals(2,:);
    robot_traj_z = zeros(size(robot_traj_x));;
    
    % Create trajectory plot
    robot_trajectory = plot3(robot_traj_x, robot_traj_y, robot_traj_z, 'w--', 'LineWidth', 1);
    
    % Total animation duration
    total_time = tspan(2) - tspan(1);
    
    % Animation loop using tic-toc
    t_start = tic; % Start timer
    
    while true
        % Elapsed time since animation started, adjusted by time scale
        elapsed_time = toc(t_start) * p.time_scale;
        
        % Find current time in the simulation
        current_time = tspan(1) + elapsed_time;
        
        % Check if animation is complete
        if current_time >= tspan(2)
            break;
        end
        
        % Interpolate the solution at the current time step
        z_current = deval(solution, current_time);
        
        % Extract current state variables
        x_current = z_current(1);
        y_current = z_current(2); % Assuming y is second state
        phi_current = z_current(4); % Pendulum angle
        psi_current = z_current(5); % Heading angle
        
        % Update basis vectors
        lambda = [cos(psi_current); sin(psi_current); 0];
        nhat = [-sin(psi_current); cos(psi_current); 0];
        er = [-sin(phi_current)*cos(psi_current); 
              -sin(phi_current)*sin(psi_current); 
              cos(phi_current)];
        et = [-cos(phi_current)*cos(psi_current); 
              -cos(phi_current)*sin(psi_current); 
              -sin(phi_current)];
        
        % Current position of robot center
        robot_pos = [x_current; y_current; 0];
        
        % Update wheel positions
        left_wheel_pos = robot_pos + dw * nhat;
        right_wheel_pos = robot_pos - dw * nhat;
        
        % Update wheel visualizations
        for i = 1:length(theta_wheel)
            % Wheel offset from center
            wheel_offset = wheel_radius * (cos(theta_wheel(i)) * lambda + sin(theta_wheel(i)) * [0; 0; 1]);
            
            % Left wheel
            left_wheel_point = left_wheel_pos + wheel_offset;
            left_wheel_x(i) = left_wheel_point(1);
            left_wheel_y(i) = left_wheel_point(2);
            left_wheel_z(i) = left_wheel_point(3);
            
            % Right wheel
            right_wheel_point = right_wheel_pos + wheel_offset;
            right_wheel_x(i) = right_wheel_point(1);
            right_wheel_y(i) = right_wheel_point(2);
            right_wheel_z(i) = right_wheel_point(3);
        end
        
        
        % Update chassis center
        chassis_center = robot_pos;
        
        % Update pendulum rectangle - now properly aligned with chassis center
        % Base corners
        bl = chassis_center - (pendulum_width/2) * nhat - pendulum_thickness/2 * et;  % Bottom left
        br = chassis_center + (pendulum_width/2) * nhat - pendulum_thickness/2 * et;  % Bottom right
        
        % Top corners (d along er direction)
        tl = bl + d * er;  % Top left
        tr = br + d * er;  % Top right
        
        % Update all visual elements
        set(left_wheel, 'XData', left_wheel_x, 'YData', left_wheel_y, 'ZData', left_wheel_z);
        set(right_wheel, 'XData', right_wheel_x, 'YData', right_wheel_y, 'ZData', right_wheel_z);
        
        % Update chassis
        % set(chassis, 'XData', [left_wheel_pos(1), right_wheel_pos(1)], ...
        %             'YData', [left_wheel_pos(2), right_wheel_pos(2)], ...
        %             'ZData', [left_wheel_pos(3), right_wheel_pos(3)]);
        
        % Update axles
        % set(left_axle, 'XData', [left_wheel_pos(1), left_wheel_pos(1)], ...
        %               'YData', [left_wheel_pos(2), left_wheel_pos(2)], ...
        %               'ZData', [left_wheel_pos(3)-wheel_radius/2, left_wheel_pos(3)+wheel_radius/2]);
        % 
        % set(right_axle, 'XData', [right_wheel_pos(1), right_wheel_pos(1)], ...
        %                'YData', [right_wheel_pos(2), right_wheel_pos(2)], ...
        %                'ZData', [right_wheel_pos(3)-wheel_radius/2, right_wheel_pos(3)+wheel_radius/2]);
        
        % Update pendulum rectangle
        new_verts = [bl'; br'; tr'; tl'];
        set(pendulum_rect, 'Vertices', new_verts);
        
        % Optionally move the camera to follow the robot
        if move_frame == 1
            xlim([robot_pos(1)-5, robot_pos(1)+5]);
            ylim([robot_pos(2)-5, robot_pos(2)+5]);
        end
        
        % Pause to create animation effect
        drawnow;
    end
    
    % Add title and labels
    title('2D Robot with Pendulum Animation');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    hold off;
    retval = 1; % Return value
end