function simulate_leggedWheelchair()
    setpath  % add subfolders
    
    close all;

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

    % Ground contact properties
    restitution_coeff = 0.1;
    friction_coeff = 0.3;
    ground_height = 0;

    %% Parameter vector
    % p   = [lU phiU mU I_U m1 m2 ma mb I_1 I_2 I_5 I_A I_B l1 l2 b l_c1 ...
    %     l_c2 l_cb r Ir N g]';
    p = [m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';


    %% Create gait generator
    setpath;
    tStance = 0.7; % seconds
    gdPen = 0.25; % meters
    avgVel = [2; 0]; % m/s
    nomHip = [0.5; 0.15];
    ctrlPts = [0.00 0.10 0.50 0.90 1.00;
               0.00 1.00 0.50 1.00 0.00];
    
    traj_obj = GaitGenerator(ctrlPts, nomHip, tStance, gdPen, avgVel);
    
%     traj_obj = [];

    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 10;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    % order of generalized coordinates [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];
    z0 = [pi/8; pi/4; pi/8; pi/4; pi/6; 0.5; 0.15; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    z0 = [7*pi/24; pi/4; pi/24; pi/4; pi/8; 0.5; 0.05; 0; 0; 0; 0; 0; 0; 0; 0; 0];
%     z0 = [pi/8; pi/4; pi/8; pi/4; pi/8; 0.5; 0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    q = 1:length(z0)/2; dq = (length(z0)/2 + 1):(length(z0));
    z_out = zeros(length(z0), num_steps);
    z_out(:, 1) = z0;
    
    for i = 1:num_steps - 1
        % Compute acceleration and pre-impact velocity without constraints
        dz = dynamics(tspan(i), z_out(:, i), p, traj_obj);

        % Compute pre-impact velocity
        qdot_minus = z_out(dq,i) + dz(dq) * dt;
        z_out(dq,i) = qdot_minus;

        % Check for contact and update velocity as needed
        qdot_plus = discrete_impact_contact(z_out(:,i), p, restitution_coeff, ...
            friction_coeff, ground_height);

        % Velocity update with dynamics
        z_out(dq, i + 1) = qdot_plus;
        
        % Position update
        z_out(q, i + 1) = z_out(q, i) + qdot_plus * dt;
                
    end

    %% Compute foot position and velocity over time
    rFeet = zeros(2,2,length(tspan));
    vFeet = zeros(2,2,length(tspan));
    for i = 1:length(tspan)
        rFeet(:,:,i) = position_feet(z_out(:,i),p);
        vFeet(:,:,i) = velocity_feet(z_out(:,i),p);
    end

    % Plot position of feet over time
    figure(1)
    plot(tspan,squeeze(rFeet(1,1,:)),'LineWidth',2) % left, x
    hold on
    plot(tspan,squeeze(rFeet(2,1,:)),'LineWidth',2) % left, y
    plot(tspan,squeeze(rFeet(1,2,:)),'LineWidth',2) % right, x
    plot(tspan,squeeze(rFeet(2,2,:)),'LineWidth',2) % right, y

    xlabel('Time (s)'); ylabel('Position (m)'); legend({'L_x','L_y','R_x','R_y'});
    title('Feet Position')

    % Plot velocity of feet over time
    figure(2)
    plot(tspan,squeeze(vFeet(1,1,:)),'LineWidth',2) % left, x
    hold on
    plot(tspan,squeeze(vFeet(2,1,:)),'LineWidth',2) % left, y
    plot(tspan,squeeze(vFeet(1,2,:)),'LineWidth',2) % right, x
    plot(tspan,squeeze(vFeet(2,2,:)),'LineWidth',2) % right, y

    xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend({'L_dx','L_dy','R_dx','R_dy'});
    title('Feet Velocity')

    %% Animate Solution
    figure(3); clf;
    
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
    plot([0 1],[ground_height ground_height],'k'); 
    xlabel('x')
    ylabel('y')
    
    h_title = title('t = 0.0s');
    
    axis equal
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
        rW = keypoints(:, 14);

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
            waitforbuttonpress % don't simulate until you click figure
        end

        pause(.01)
        delete(wheelHead)
    end
wheelHead = viscircles([rA(1) rA(2); rU'], [r 0.01]);
end

function dz = dynamics(t, z, p, traj_obj)
    % Get mass matrix
    A = A_leggedWheelchair(z, p);
    
    % Get forces
%     tau = control_law(t, z, p, traj_obj);
    tau = [0 0 0 0]';
    b = b_leggedWheelchair(z, tau, p);
    
    % Solve for qdd
    qdd = A\b;
    
    % Form dz
    dz = 0*z;
    dz(1:8) = z(9:16);
    dz(9:16) = qdd;
end

function tau = control_law(t, z, p, traj_obj)
    % Controller gains (same for each foot)
    K_x = 150.; % Spring stiffness X
    K_y = 150.; % Spring stiffness Y
    D_x = 10.;  % Damping X
    D_y = 10.;  % Damping Y

    % Desired position of feet relative to hip
    [footTraj_hip, inContact] = traj_obj.footPatternGenerator(t);
    % foot traj returns Lx, Ly, Rx, Ry positions of feet

    % Convert desired trajectory into global coordinates based on actual
    % hip position
    rHip = position_hip(z,p);
    rllEd = rHip + footTraj_hip(1:2);
    rrlEd = rHip + footTraj_hip(3:4);

    % Fill in velocity and acceleration here when available
    
    % Actual position and velocity 
    rE = position_feet(z,p);
    vE = velocity_feet(z,p);

    % Compute virtual forces
%     f  = [K_x * (rllEd(1) - rE(1) ) + D_x * (drllEd(1) - vE(1) ) ;  % Lx
%           K_y * (rllEd(2) - rE(2) ) + D_y * (drllEd(2) - vE(2) ) ;  % Ly
%           K_x * (rrlEd(1) - rE(3) ) + D_x * (drrlEd(1) - vE(3) ) ;  % Rx
%           K_y * (rrlEd(2) - rE(4) ) + D_y * (drrlEd(2) - vE(4) ) ;];% Ry
    f  = [K_x * (rllEd(1) - rE(1) ) ;  % Lx
          K_y * (rllEd(2) - rE(2) ) ;  % Ly
          K_x * (rrlEd(1) - rE(3) ) ;  % Rx
          K_y * (rrlEd(2) - rE(4) ) ;];% Ry
    
    %% Task-space compensation and feedforward
    % Get operational space terms
    A = A_leggedWheelchair(z,p);
    C = Corr_leg(z,p);
    G = Grav_leg(z,p);
    J  = jacobian_feet(z,p);
    dJ = jacobian_dot_feet(z,p);
    J = J(1:4, :); % exclude wheel terms in Jacobian
    dJ = dJ(1:4, :);
    dq = z(9:16);

    % Map to joint torques  
    L = inv(J * inv(A) * J');
    mu = L * J * inv(A) * C - L * dJ * dq;
    rho = L * J * inv(A) * G;

%     tau = J' * (L * (aEd + f) + mu + rho);
    tau = J' * (L * f + mu + rho);

%     tau = [0 0 0 0]'; % placeholder
end

function qdot = discrete_impact_contact(z, p, rest_coeff, fric_coeff, yC)
    % Actual position and velocity of feet and wheel
    rFeet = position_feet(z, p);
    rllE = rFeet(:,1);
    rrlE = rFeet(:,2);
    rWheel = position_wheel(z, p);

    drFeet = velocity_feet(z, p);
    drllE = drFeet(:,1);
    drrlE = drFeet(:,2);
    drWheel = velocity_wheel(z, p);

    yl = rllE(2);
    yldot = drllE(2);
    yr = rrlE(2);
    yrdot = drrlE(2);
    yw = rWheel(2);
    ywdot = drWheel(2);

    qdot = z(9:16);

    % Compute the height of the feet and wheel relative to the ground
    C_yl = yl - yC;
    C_yr = yr - yC;
    C_yw = yw - yC;
    Cy = [C_yl; C_yr; C_yw];

    % Compute the velocity of the feet relative to the ground
    C_yl_dot = yldot;
    C_yr_dot = yrdot;
    C_yw_dot = ywdot;
    C_y_dot = [yldot; yrdot; ywdot];

    % Choose which legs to model contact with (for debugging & comparison)
%     leg = 'left';
%     leg = 'right';
%     leg = 'both';
    leg = 'both and wheel';

    if strcmp(leg, 'left')

        % Check constraints
        if ((C_yl > 0 || C_yl_dot > 0))
            % If constraints aren't violated, don't update qdot
            return
        else
            % Compute vertical impulse force
            A = A_leggedWheelchair(z,p);
            J  = jacobian_feet(z,p);
            J_c_yl = J(2,:); % Jacobian of left foot in y-directionn
            L_c_yl = inv(J_c_yl * inv(A) * J_c_yl'); % operational space mass matrix in y-direction
    
            % Vertical forces on feet
            F_c_yl = L_c_yl * (-rest_coeff * C_yl_dot - J_c_yl * qdot);
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_yl'*F_c_yl);
    
            % Compute tangential impulse forces
            J_c_xl = J(1,:); % Jacobian of left foot in x-direction
            L_c_xl = inv(J_c_xl * inv(A) * J_c_xl');
            F_c_xl = L_c_xl * (0 - J_c_xl * qdot);
    
            % Truncate F_c_x if it is outside of friction cone for each foot
            if F_c_xl > fric_coeff * F_c_yl
                F_c_xl = fric_coeff * F_c_yl;
            elseif F_c_xl < -fric_coeff * F_c_yl
                F_c_xl = -fric_coeff * F_c_yl;
            end
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_xl'*F_c_xl);
        end

    elseif strcmp(leg, 'right')

        % Check constraints
        if ((C_yr > 0 || C_yr_dot > 0))
            % If constraints aren't violated, don't update qdot
            return
        else
            % Compute vertical impulse force
            A = A_leggedWheelchair(z,p);
            J  = jacobian_feet(z,p);
            J_c_yr = J(4,:); % Jacobian of right foot in y-direction
            L_c_yr = inv(J_c_yr * inv(A) * J_c_yr'); % operational space mass matrix in y-direction
    
            % Vertical forces on feet
            F_c_yr = L_c_yr * (-rest_coeff * C_yr_dot - J_c_yr * qdot);
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_yr'*F_c_yr);

            % Compute tangential impulse forces
            J_c_xr = J(3,:); % Jacobian of right foot in x-direction
            L_c_xr = inv(J_c_xr * inv(A) * J_c_xr');
            F_c_xr = L_c_xr * (0 - J_c_xr * qdot);
    
            % Truncate F_c_x if it is outside of friction cone for each foot
            if F_c_xr > fric_coeff * F_c_yr
                F_c_xr = fric_coeff * F_c_yr;
            elseif F_c_xr < -fric_coeff * F_c_yr
                F_c_xr = -fric_coeff * F_c_yr;
            end
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_xr'*F_c_xr);
        end

    elseif strcmp(leg, 'both')

         % Check constraints
        if ((C_yl > 0 || C_yl_dot > 0) && (C_yr > 0 || C_yr_dot > 0))
            % If constraints aren't violated, don't update qdot
            return
        else
            % Compute vertical impulse force
            A = A_leggedWheelchair(z,p);
            J  = jacobian_feet(z,p);
            J_c_y = vertcat(J(2,:), J(4,:));
            L_c_y = inv(J_c_y * inv(A) * J_c_y');
            Cy = [C_yl; C_yr];
            C_y_dot = [yldot; yrdot];
    
            % Vertical forces on feet
            F_c_y = L_c_y * (-rest_coeff * C_y_dot - J_c_y * qdot);
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_y'*F_c_y);
    
            % Compute tangential impulse forces
            J_c_x = vertcat(J(1,:), J(3,:));
            L_c_x = inv(J_c_x * inv(A) * J_c_x');
            F_c_x = L_c_x * (0 - J_c_x * qdot);
    
            % Truncate F_c_x if it is outside of friction cone for each foot
            if F_c_x(1) > fric_coeff * F_c_y(1)
                F_c_x(1) = fric_coeff * F_c_y(1);
            elseif F_c_x(1) < -fric_coeff * F_c_y(1)
                F_c_x(1) = -fric_coeff * F_c_y(1);
            end
    
            if F_c_x(2) > fric_coeff * F_c_y(2)
                F_c_x(2) = fric_coeff * F_c_y(2);
            elseif F_c_x(2) < -fric_coeff * F_c_y(2)
                F_c_x(2) = -fric_coeff * F_c_y(2);
            end
    
            % Update qdot
            qdot = qdot + inv(A) * J_c_x' * F_c_x;
        end

    elseif strcmp(leg, 'both and wheel')

         % Check constraints
        if ((C_yl > 0 || C_yl_dot > 0) && (C_yr > 0 || C_yr_dot > 0) && ...
                (C_yw > 0 || C_yw_dot > 0))
            % If constraints aren't violated, don't update qdot
            return
        else
            % Compute vertical impulse force
            A = A_leggedWheelchair(z,p);
            J  = jacobian_feet(z,p);
            J_c_y = vertcat(J(2,:), J(4,:), J(6,:));
            L_c_y = inv(J_c_y * inv(A) * J_c_y');
    
            % Vertical forces on feet
            F_c_y = L_c_y * (-rest_coeff * C_y_dot - J_c_y * qdot);
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_y'*F_c_y);
    
            % Compute tangential impulse forces
            J_c_x = vertcat(J(1,:), J(3,:), J(5,:));
            L_c_x = inv(J_c_x * inv(A) * J_c_x');
            F_c_x = L_c_x * (0 - J_c_x * qdot);
    
            % Truncate F_c_x if it is outside of friction cone for each
            % point of contact
            % left foot
            if F_c_x(1) > fric_coeff * F_c_y(1)
                F_c_x(1) = fric_coeff * F_c_y(1);
            elseif F_c_x(1) < -fric_coeff * F_c_y(1)
                F_c_x(1) = -fric_coeff * F_c_y(1);
            end
    
            % right foot
            if F_c_x(2) > fric_coeff * F_c_y(2)
                F_c_x(2) = fric_coeff * F_c_y(2);
            elseif F_c_x(2) < -fric_coeff * F_c_y(2)
                F_c_x(2) = -fric_coeff * F_c_y(2);
            end

            % wheel
            if F_c_x(3) > fric_coeff * F_c_y(3)
                F_c_x(3) = fric_coeff * F_c_y(3);
            elseif F_c_x(3) < -fric_coeff * F_c_y(3)
                F_c_x(3) = -fric_coeff * F_c_y(3);
            end
    
            % Update qdot
            qdot = qdot + inv(A) * J_c_x' * F_c_x;
        end

    end
end