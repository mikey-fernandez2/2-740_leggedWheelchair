function simulate_wheelchair()
    setpath  % add subfolders

    %% Define fixed paramters
    % Lengths in meters, angles in radians, mass in kg
    lU = 0.110; % distance from wheel center to "user" CoM
    phiU = deg2rad(80); % angle from axis to "user" CoM
    mU = 0.03; % 30 grams (2 oz.)
    I_U = 1/12 * mU * lU^2; % approximate as slender rod
    m1 = .0393 + .2;
    m2 = .0368 + .00783 + .0155; % combine the masses of links 2, 3, and 4 from lab
    m5 = 0.0424 * 12 / 2.205; % 12" length of 80/20 1010 profile
    ma = 0.200; % 100 grams per wheel (6 oz. each) - weigh them!
    mb = 2.0; % total guess; mass of motors plus mounting hardware
    b = 0.267;
    r = 0.035;

    l1 = 0.096; % same as l_AC from lab (not quite accurate)
    l2 = 0.091; % same as l_DE from lab (not quite accurate)
    l_c1 = 0.0344; % l_B_m2 from lab
    l_c2 = 0.0610; % l_C_m4 from lab (revisit this)
    l_cb = 0.1524; % 6" (doesn't account for amount hanging off past connection points)

    I_1 = 25.1 * 10^-6;
    I_2 = (53.5 + 9.25 + 22.176) * 10^-6;
    I_5 = 1/12 * m5 * b;
    I_A = 0.5 * ma * r^2; % thin solid disk
    I_B = 1 * 10^-3; % truly a random guess. Need to do more calcs
    
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81;

    %% Parameter vector
    p   = [lU phiU mU I_U m1 m2 ma mb I_1 I_2 I_5 I_A I_B l1 l2 b l_c1 ...
        l_c2 l_cb r Ir N g]';
    
    %% Perform Dynamic simulation
    dt = 0.001;
    tf = 10;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    z0 = [-pi/4; 0; pi/4; pi/2; pi/6; 0.5; 0.5; 0;
           0; 0; 0; 0; 0; 0; 0; 0];
    z_out = zeros(16,num_steps);
    z_out(:,1) = z0;
    
    for i = 1:num_steps - 1

        dz = dynamics(z_out(:,i), p);

        z_out(9:16,i+1) = z_out(9:16,i) + dz(9:16)*dt;
        z_out(1:8,i+1) = z_out(1:8,i) + z_out(9:16,i+1)*dt;
                
    end

    %% Animate Solution
    figure(1); clf;
    
    % Prepare plot handles
    hold on
    h_AB = plot([0],[0],'LineWidth',2);
    h_AU = plot([0],[0],'LineWidth',2);
    h_BC = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_DF = plot([0],[0],'LineWidth',2);
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
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
        
        % Get vectors to keypoints
        rA = keypoints(:,1);
        rB = keypoints(:,2);
        rC = keypoints(:,3);
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        rF = keypoints(:,6);
        rU = keypoints(:,7);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title

        % plot circle
        wheel = viscircles([rA(1) rA(2)], r);

        % Plot rod
        set(h_AB,'XData' , [rA(1) rB(1)] );
        set(h_AB,'YData' , [rA(2) rB(2)] );

        % Plot user
        set(h_AU,'XData' , [rA(1) rU(1)] );
        set(h_AU,'YData' , [rA(2) rU(2)] );

        % Plot legs
        set(h_BC,'XData' , [rB(1) rC(1)] );
        set(h_BC,'YData' , [rB(2) rC(2)] );

        set(h_CE,'XData' , [rC(1) rE(1)] );
        set(h_CE,'YData' , [rC(2) rE(2)] );

        set(h_BD,'XData' , [rB(1) rD(1)] );
        set(h_BD,'YData' , [rB(2) rD(2)] );

        set(h_DF,'XData' , [rD(1) rF(1)] );
        set(h_DF,'YData' , [rD(2) rF(2)] );

        pause(.01)
        delete(wheel)   
    end
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