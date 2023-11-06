function simulate_leggedWheelchair()
    setpath  % add subfolders
    
    restitution_coeff = 0.1;
    friction_coeff = 0.3;
    ground_height = 0;

    %% Define fixed paramters
    % Lengths in meters, angles in radians, mass in kg
    lU = 0.110; % distance from wheel center to "user" CoM
    phiU = deg2rad(80); % angle from axis to "user" CoM
    mU = 0.03; % 30 grams (2 oz.)
    I_U = 1/12 * mU * lU^2; % approximate as slender rod
    m1 = .0393 + .2; % from lab
    m2 = .0368; % from lab
    m3 = .00783; % from lab
    m4 = .0155; % from lab
    m5 = 0.0424 * 12 / 2.205; % 12" length of 80/20 1010 profile
    ma = 0.200; % 100 grams per wheel (6 oz. each) - weigh them!
    mb = m5 + 2.0; % total guess; mass of motors plus mounting hardware
    b = 0.267; % 12" 80/20
   
    r = 0.035; % wheels from 2.007 lab

    l_OA = 0.011; % same as l_OA from lab (not quite accurate)
    l_OB = 0.042; % same as l_OB from lab (not quite accurate)
    l_AC = 0.096; % same as l_AC from lab (not quite accurate)
    l_DE = 0.091; % same as l_DE from lab (not quite accurate)
    l_O_m1 = 0.032; % l_O_m1
    l_B_m2 = 0.0344; % l_B_m2
    l_A_m3 = 0.0622; % l_A_m3 from lab
    l_C_m4 = 0.0610; % l_C_m4 from lab (revisit this)
    l_cb = 0.1524; % 6" (doesn't account for amount hanging off past connection points)

    I1 = 25.1 * 10^-6;
    I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;
    I4 = 22.176 * 10^-6;
    I_A = 0.5 * ma * r^2; % thin solid disk
    I_B = 1 * 10^-3; % truly a random guess. Need to do more calcs
    
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;

    %% Parameter vector
    % p   = [lU phiU mU I_U m1 m2 ma mb I_1 I_2 I_5 I_A I_B l1 l2 b l_c1 ...
    %     l_c2 l_cb r Ir N g]';
    p = [m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';

    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 10;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    % order of generalized coordinates [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];
    z0 = [pi/8; pi/4; pi/8; pi/4; pi/4; 0.5; 0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    q = 1:length(z0)/2; dq = (length(z0)/2 + 1):(length(z0));
    z_out = zeros(length(z0), num_steps);
    z_out(:, 1) = z0;
    
    for i = 1:num_steps - 1
        dz = dynamics(z_out(:, i), p);

        z_out(dq, i + 1) = z_out(dq, i) + dz(dq)*dt;
        z_out(q, i + 1) = z_out(q, i) + z_out(dq, i + 1)*dt;
                
    end

    %% Animate Solution
    figure(1); clf;
    
    % Prepare plot handles
    hold on
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
    xlabel('x')
    ylabel('y');exit
    
    h_title = title('t = 0.0s');
    
    axis equal
%     axis([-0.5 0.5 -0.5 0.5]);
    skip_frame = 100;
    
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

        % plot circle
        wheelHead = viscircles([rA(1) rA(2); rU'], [r 0.01]);

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

        if i == skip_frame
            waitforbuttonpress % dont simulate until you start
        end
        pause(.01)
        delete(wheelHead)
    end
wheelHead = viscircles([rA(1) rA(2); rU'], [r 0.01]);
end

function dz = dynamics(z,p)
    % Get mass matrix
    A = A_leggedWheelchair(z,p);
    
    % Get forces
    u = [0 0 0 0]';
    b = b_leggedWheelchair(z, u, p);
    
    % Solve for qdd
    qdd = A\b;
    
    % Form dz
    dz = 0*z;
    dz(1:8) = z(9:16);
    dz(9:16) = qdd;
end