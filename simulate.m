function [z_out, dz_out] = simulate(sim, p, ctrlStruct)
% simulate the system with the simulation parameters given in sim, the
% model parameters given in p, and the desired gait/controller parameters
% in ctrlStruct

    dt = sim.dt; num_steps = sim.num_steps; tspan = sim.tspan;
    % order of generalized coordinates [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];
    z0 = sim.z0;
    q = 1:length(z0)/2; dq = (length(z0)/2 + 1):(length(z0));
    z_out = zeros(length(z0), num_steps);
    dz_out = zeros(length(z0), num_steps);
    z_out(:, 1) = z0;
    
    for i = 1:num_steps - 1
        % Compute acceleration and pre-impact velocity without constraints
        dz = dynamics(tspan(i), z_out(:, i), p, ctrlStruct);
        dz_out(:, i + 1) = dz;

        % Compute pre-impact velocity
        qdot_minus = z_out(dq, i) + dz(dq) * dt;
        z_out(dq, i) = qdot_minus;

        % Check for contact and update post-impact velocity as needed
        qdot_plus = discrete_impact_contact(z_out(:, i), p, sim);
        z_out(dq, i) = qdot_plus;

        % Check for joint limits and update velocity as needed
        qdot_plus = joint_limit_constraint(z_out(:,i), sim); % is this potentially a problem with joint limits overriding contact?

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
    K = ctrlStruct.K*eye(4); % Spring stiffness
    D = ctrlStruct.D*eye(4);  % Damping

    % identify state, parameters
    % Task-space compensation and feedforward
    % A = A_leggedWheelchair(z, p);
    % C = Corr_leg(z, p);
    % G = Grav_leg(z, p);
    J  = jacobian_feet(z, p);
    % dJ = jacobian_dot_feet(z, p);
    J = J(1:4, :); % exclude wheel terms in Jacobian
    % dJ = dJ(1:4, :);
    dq = z(9:16);

    % Desired kinematics of feet relative to hip
    [kinematicsDes, inContact] = ctrlStruct.gait.footPatternGenerator(t);
    % kinematicsDes is [Lx; Ly; Rx; Ry; Ldx; Ldy; Rdx; Rdy; Lddx; Lddy; Rddx; Rddy]

    posDes = kinematicsDes(1:4);
    velDes = kinematicsDes(5:8);
    accDes = kinematicsDes(9:12);

    % by gait design/assumption, we are always walking - there will be at
    % least one foot in contact with the ground, so we can get the hip
    % position relative to the foot in contact
    % then get feet positions, relative to the **ground** from the hip
    %   - foot in contact is at -hipPos
    %   - the other foot is at -rHipFeet(other) - hipPos
    rHip_feet = hip_relative_feet(z, p);
    posCur = -rHip_feet;
    velCur = J(:, 1:4)*dq(1:4); % foot velocity, relative to the hip (so ignore other terms)

    % get actual hip position
    actualHipHeight = sum(rHip_feet([2 4]).*inContact);
    hipPosDes = ctrlStruct.gait.nomHip(2);
    hipPosError = actualHipHeight - hipPosDes;
    hipFeedforward = [0; hipPosError; 0; hipPosError].*repelem(inContact, 2);%.*(hipPosError < 0);

    % get the errors to compute virtual forces
    posErr = posDes - posCur;% + hipFeedforward;
    velErr = velDes - velCur;

    f = accDes + K*posErr + D*velErr;
    tau = J(:, 1:4)'*f;
end

function qdot = discrete_impact_contact(z, p, sim)
    rest_coeff = sim.restitution_coeff;
    fric_coeff = sim.friction_coeff;
    wheel_fric = sim.wheel_friction;
    yC = sim.ground_height;
    
    % Actual position and velocity of feet, wheel, and hip
    rFeet = position_feet(z, p); rllE = rFeet(:, 1); rrlE = rFeet(:, 2); rWheel = position_wheel(z, p);

    drFeet = velocity_feet(z, p); drllE = drFeet(:, 1); drrlE = drFeet(:, 2); drWheel = velocity_wheel(z, p);

    yl = rllE(2); yldot = drllE(2);
    yr = rrlE(2); yrdot = drrlE(2);
    yw = rWheel(2); ywdot = drWheel(2);

    qdot = z(9:16);

    % Compute the height of the feet, wheel, and hip relative to the ground
    C_yl = yl - yC;
    C_yr = yr - yC;
    C_yw = yw - yC;
    C_y = [C_yl; C_yr; C_yw];

    % Compute the velocity of the feet relative to the ground
    C_yl_dot = yldot;
    C_yr_dot = yrdot;
    C_yw_dot = ywdot;
    C_y_dot = [C_yl_dot; C_yr_dot; C_yw_dot];

    constraintsViolated = ~(C_y > 0 | C_y_dot > 0);
    frictionCoeffs = [fric_coeff; fric_coeff; wheel_fric];

     % Check constraints
    if any(constraintsViolated)
        F_thresh = 0.001; % convergence force threshold
        max_iters = 10;

        rowsY = [2 4 6]; rowsX = [1 3 5];
        for iter = 1:max_iters
            allForces_y = zeros(size(constraintsViolated)); allVel_y = zeros(size(constraintsViolated));
            for contact = 1:length(constraintsViolated) % loop through all potential contact points
                if constraintsViolated(contact) % only update force for points that are in contact        
                    % Compute vertical impulse force on contact
                    A = A_leggedWheelchair(z, p);
                    J = jacobian_feet(z, p);
                    J_c_y = J(rowsY(contact), :); % just select row of Jacobian corresponding to current contact
                    L_c_y_inv = J_c_y*(A\J_c_y');
                    F_c_y = L_c_y_inv\(-rest_coeff*C_y_dot(contact)' - J_c_y*qdot);
                    allForces_y(contact) = F_c_y;

                    % Update qdot with vertical reaction forces
                    qdot = qdot + (A\(J_c_y'*F_c_y));
                    vcz = J_c_y*qdot; % compute vertical velocity of contact after updating qdot
                    allVel_y(contact) = vcz;
            
                    % Compute tangential impulse forces
                    J_c_x = J(rowsX(contact), :);
                    L_c_x_inv = J_c_x*(A\J_c_x');
                    F_c_x = L_c_x_inv\(0 - J_c_x*qdot);

                    frictionCone = abs(F_c_y*frictionCoeffs(contact));

                    % Truncate F_c_x if it is outside of friction cone
                    F_c_x = max(-frictionCone, min(frictionCone, F_c_x));
            
                    % Update qdot
                    qdot = qdot + (A\J_c_x'*F_c_x);
                    z = [z(1:8); qdot];
                end
            end

            if all(allVel_y >= 0) && all(allForces_y >= 0) && all(abs(allVel_y.*allForces_y) < F_thresh)
                break
            end
        end
    end
end

function qdot = joint_limit_constraint(z, sim)
    % Joint limits for a given leg (visually estimated)
    qC1_min = sim.qC1_min;
    qC1_max = sim.qC1_max;
    qC2_min = sim.qC2_min;
    qC2_max = sim.qC2_max;
    
    % Name variables for readability
    q = z(1:4); qdotLeg = z(9:12);

    % joint limits, as an array
    maxLimits = [-qC1_min; -qC2_min; qC1_max; qC2_max];
    minLimits = [-qC1_max; -qC2_max; qC1_min; qC2_min];

    % check constraints
    constraintsViolated = (q > maxLimits & qdotLeg < 0) | (q < minLimits & qdotLeg > 0) | (q > maxLimits & q < minLimits);
    qdotUpdate = [~constraintsViolated; ones(4, 1)];
    qdot = z(9:16).*qdotUpdate;
end