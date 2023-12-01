function animateSol(sim, p, z_out)
%% Animate Solution
    tspan = sim.tspan; num_steps = sim.num_steps;

    figure(111); clf; hold on 
    % Prepare plot handles
    h_AB = plot([0], [0], 'LineWidth', 2);
    h_AU = plot([0], [0], 'LineWidth', 2);
    h_l1 = plot([0], [0], 'LineWidth', 2);
    h_l2 = plot([0], [0], 'LineWidth', 2);
    h_l3 = plot([0], [0], 'LineWidth', 2);
    h_l4 = plot([0], [0], 'LineWidth', 2);
    h_r1 = plot([0], [0], 'LineWidth', 2);
    h_r2 = plot([0], [0], 'LineWidth', 2);
    h_r3 = plot([0], [0], 'LineWidth', 2);
    h_r4 = plot([0], [0], 'LineWidth', 2);
    plot([-0.5 1.5], [sim.ground_height sim.ground_height],'k'); 
    xlabel('x')
    ylabel('y')
    
    h_title = title('t = 0.0s');
    
    axis equal
    skip_frame = 10; % adjust animation frame rate
    
    % Step through and update animation
    for i = 1:num_steps
        if mod(i, skip_frame)
            continue
        end

        % interpolate to get state at current time.
        t = tspan(i);
        z = z_out(:, i);
        keypoints = keypoints_leggedWheelchair(z, p);
        
        % keypoints = [rA rB  rrlA  rrlB  rrlC  rrlD  rrlE  rllA  rllB  rllC  rllD  rllE  rU ];
        % Get vectors to keypoints
        rA = keypoints(:, 1);
        rB = keypoints(:, 2);
        rllA = keypoints(:, 3);
        rllB = keypoints(:, 4);
        rllC = keypoints(:, 5);
        rllD = keypoints(:, 6);
        rllE = keypoints(:, 7);
        rrlA = keypoints(:, 8);
        rrlB = keypoints(:, 9);
        rrlC = keypoints(:, 10);
        rrlD = keypoints(:, 11);
        rrlE = keypoints(:, 12);
        rU = keypoints(:, 13);

        set(h_title, 'String', sprintf('t = %.2f', t)); % update title

        % plot circles
        wheelHead = viscircles([rA(1) rA(2); rU'], [p(23) 0.01]);

        % Plot rod
        set(h_AB, 'XData', [rA(1) rB(1)]);
        set(h_AB, 'YData', [rA(2) rB(2)]);

        % Plot user
        set(h_AU, 'XData', [rA(1) rU(1)]);
        set(h_AU, 'YData', [rA(2) rU(2)]);

        % Plot legs
        set(h_l1, 'XData', [rB(1) rllB(1)]);
        set(h_l1, 'YData', [rB(2) rllB(2)]);
        set(h_l2, 'XData', [rllA(1) rllC(1)]);
        set(h_l2, 'YData', [rllA(2) rllC(2)]);
        set(h_l3, 'XData', [rllB(1) rllD(1)]);
        set(h_l3, 'YData', [rllB(2) rllD(2)]);
        set(h_l4, 'XData', [rllC(1) rllE(1)]);
        set(h_l4, 'YData', [rllC(2) rllE(2)]);

        set(h_r1, 'XData', [rB(1) rrlB(1)]);
        set(h_r1, 'YData', [rB(2) rrlB(2)]);
        set(h_r2, 'XData', [rrlA(1) rrlC(1)]);
        set(h_r2, 'YData', [rrlA(2) rrlC(2)]);
        set(h_r3, 'XData', [rrlB(1) rrlD(1)]);
        set(h_r3, 'YData', [rrlB(2) rrlD(2)]);
        set(h_r4, 'XData', [rrlC(1) rrlE(1)]);
        set(h_r4, 'YData', [rrlC(2) rrlE(2)]);

        pause(.01)
        delete(wheelHead)
    end
    wheelHead = viscircles([rA(1) rA(2); rU'], [p(23) 0.01]);
end