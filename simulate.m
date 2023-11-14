function [z_out, dz_out] = simulate(sim, p, ctrlStruct)
% simulate the system with the simulation parameters given in sim, the
% model parameters given in p, and the desired gait/controller parameters
% in ctrlStruct

    dt = sim.dt; tf = sim.tf; num_steps = sim.num_steps; tspan = sim.tspan;
    % order of generalized coordinates [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];
    z0 = sim.z0;
    q = 1:length(z0)/2; dq = (length(z0)/2 + 1):(length(z0));
    z_out = zeros(length(z0), num_steps);
    dz_out = zeros(length(z0), num_steps);
    z_out(:, 1) = z0;
    
    for i = 1:num_steps - 1
        % Compute acceleration and pre-impact velocity without constraints
        dz = dynamics(tspan(i), z_out(:, i), p, ctrlStruct);
        dz_out(:, i) = dz;

        % Compute pre-impact velocity
        qdot_minus = z_out(dq, i) + dz(dq) * dt;
        z_out(dq, i) = qdot_minus;

        % Check for contact and update velocity as needed
        qdot_plus = discrete_impact_contact(z_out(:, i), p, restitution_coeff, ...
            friction_coeff, ground_height);

        % Velocity update with dynamics
        z_out(dq, i + 1) = qdot_plus;
        
        % Position update
        z_out(q, i + 1) = z_out(q, i) + qdot_plus*dt;
    end
end

function dz = dynamics(t, z, p, ctrlStruct)
    % Get mass matrix
    A = A_leggedWheelchair(z, p);
    
    % Get generalized torques
    tau = control_law(t, z, p, ctrlStruct);
    b = b_leggedWheelchair(z, tau, p);
    
    % Solve for qdd
    qdd = A\b;
    
    % Form dz
    dz = 0*z;
    dz(1:8) = z(9:16);
    dz(9:16) = qdd;
end

function tau = control_law(t, z, p, ctrlStruct)
    % Controller gains (same for each foot)
    K_x = ctrlStruct.K; % Spring stiffness X
    K_y = ctrlStruct.K; % Spring stiffness Y
    D_x = ctrlStruct.D;  % Damping X
    D_y = ctrlStruct.D;  % Damping Y

    % Actual position and velocity 
    rE = position_feet(z, p);
    vE = velocity_feet(z, p);

    % Desired kinematics of feet relative to **ground directly under hip**
    [kinematicsDes, inContact] = ctrlStruct.footPatternGenerator(t);
    % kinematicsDes is [Lx; Ly; Rx; Ry; Ldx; Ldy; Rdx; Rdy; Lddx; Lddy; Rddx; Rddy]

    posDes = kinematicsDes(1:4);
    velDes = kinematicsDes(5:8);
    accDes = kinematicsDes(9:12);

    % by gait design/assumption, we are always walking - there will be at
    % least one foot in contact with the ground, so we can get the hip
    % position relative to the foot in contact
    rHip_feet = hip_relative_feet(z, p);
    assert(any(inContact), 'Neither foot is in contact with the ground - expected at least 1');
    if inContact(1) % left foot in contact
        hipPos = rHip_feet(1:2);
    else % right foot in contact
        hipPos = rHip_feet(3:4);
    end
        
    % Fill in velocity and acceleration here when available

    % Compute virtual forces
%     f  = [K_x * (rllEd(1) - rE(1) ) + D_x * (drllEd(1) - vE(1) ) ;  % Lx
%           K_y * (rllEd(2) - rE(2) ) + D_y * (drllEd(2) - vE(2) ) ;  % Ly
%           K_x * (rrlEd(1) - rE(3) ) + D_x * (drrlEd(1) - vE(3) ) ;  % Rx
%           K_y * (rrlEd(2) - rE(4) ) + D_y * (drrlEd(2) - vE(4) ) ;];% Ry
    f  = [K_x * (rllEd(1) - rE(1) ) + D_x * (0 - vE(1) ) ;  % Lx
          K_y * (rllEd(2) - rE(2) ) + D_y * (0 - vE(2) );  % Ly
          K_x * (rrlEd(1) - rE(3) ) + D_x * (0 - vE(3) );  % Rx
          K_y * (rrlEd(2) - rE(4) ) + D_y * (0 - vE(4) );];% Ry
    
    % Task-space compensation and feedforward
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
