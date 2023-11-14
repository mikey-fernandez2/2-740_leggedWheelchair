function plotFeet(sim, p, z_out)
    %% Compute foot position and velocity over time
    tspan = sim.tspan; num_steps = sim.num_steps;
    rFeet = zeros(2, 2, num_steps);
    vFeet = zeros(2, 2, num_steps);
    for i = 1:length(tspan)
        rFeet(:, :, i) = position_feet(z_out(:, i), p);
        vFeet(:, :, i) = velocity_feet(z_out(:, i), p);
    end

    % Plot position of feet over time
    figure(1); clf; hold on
    plot(tspan, squeeze(rFeet(1, 1, :)), 'LineWidth', 2) % left, x
    plot(tspan, squeeze(rFeet(2, 1, :)), 'LineWidth', 2) % left, y
    plot(tspan, squeeze(rFeet(1, 2, :)), 'LineWidth', 2) % right, x
    plot(tspan, squeeze(rFeet(2, 2, :)), 'LineWidth', 2) % right, y

    xlabel('Time (s)'); ylabel('Position (m)'); legend({'L_x','L_y','R_x','R_y'});
    title('Feet Position')

    % Plot velocity of feet over time
    figure(2); clf; hold on
    plot(tspan, squeeze(vFeet(1, 1, :)), 'LineWidth', 2) % left, x
    plot(tspan, squeeze(vFeet(2, 1, :)), 'LineWidth', 2) % left, y
    plot(tspan, squeeze(vFeet(1, 2, :)), 'LineWidth', 2) % right, x
    plot(tspan, squeeze(vFeet(2, 2, :)), 'LineWidth', 2) % right, y

    xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend({'L_dx','L_dy','R_dx','R_dy'});
    title('Feet Velocity')
end