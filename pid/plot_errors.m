function plot_errors(solution, z0, tspan, p)
    t = linspace(tspan(1), tspan(2), 1000);
    z = deval(solution, t);

    x = z(1,:);
    y = z(2,:);
    r = z(3,:);
    psi = z(5,:);

    waypoints_true = generate_waypoints(x);
    y_true = waypoints_true.y;

    subplot(2,1,1);
    plot(x, y, 'DisplayName',"Robot Trajectory");
    hold on
    plot(x,y_true, 'DisplayName',"Reference Trajectory");
    xlabel("x (m)");
    ylabel("y (m)");
    legend();
    axis equal;
    grid on;

    subplot(2,1,2);
    plot(x, y_true - y, 'DisplayName', "Error");
    xlabel("x (m)");
    ylabel("y-error (m)");
    axis equal;
    grid on;
    hold off;
end