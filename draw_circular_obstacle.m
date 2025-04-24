function draw_circular_obstacle(p_obs, radius)
    hold on
    if radius == 0
        % For point obstacles (radius = 0), draw a black circle
        plot(p_obs(1), p_obs(2), 'oblack', 'LineWidth', 3);
    else
        % For circular obstacles, draw the circle with black edge and fill
        th = 0:pi/50:2*pi;
        x_circle = radius * cos(th) + p_obs(1);
        y_circle = radius * sin(th) + p_obs(2);
        plot(x_circle, y_circle, 'k', 'LineWidth', 2); % Black edge
        fill(x_circle, y_circle, 'k', 'EdgeColor', 'k'); % Black fill and edge
    end
end