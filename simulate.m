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

        % Check for contact and update velocity as needed
        qdot_plus = discrete_impact_contact(z_out(:, i), p, sim);

        % Velocity update with dynamics
        z_out(dq, i + 1) = qdot_plus;

        % Joint limit constraint - figure out when this should be called,
        % as it may set joint velocities to 0
        qdot_plus = joint_limit_constraint(z_out(:, i), p);
        
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
    D = ctrlStruct.D;  % Damping

    % identify state, parameters
    % Task-space compensation and feedforward
    A = A_leggedWheelchair(z, p);
    C = Corr_leg(z, p);
    G = Grav_leg(z, p);
    J  = jacobian_feet(z, p);
    dJ = jacobian_dot_feet(z, p);
    J = J(1:4, :); % exclude wheel terms in Jacobian
    dJ = dJ(1:4, :);
    dq = z(9:16);

    % Desired kinematics of feet relative to **ground directly under hip**
    [kinematicsDes, inContact] = ctrlStruct.footPatternGenerator(t);
    % kinematicsDes is [Lx; Ly; Rx; Ry; Ldx; Ldy; Rdx; Rdy; Lddx; Lddy; Rddx; Rddy]

    posDes = kinematicsDes(1:4);
    velDes = kinematicsDes(5:8);
    accDes = kinematicsDes(9:12);

    % by gait design/assumption, we are always walking - there will be at
    % least one foot in contact with the ground, so we can get the hip
    % position relative to the foot in contact
    % then get feet positions, relative to the **ground** from the hip
    %   - foot in contact is at [-hipPos(1); 0]
    %   - the other foot is at hipPos - rHipFeet(other)
    rHip_feet = hip_relative_feet(z, p);
    assert(any(inContact), 'Neither foot is in contact with the ground - expected at least 1');
    if inContact(1) % left foot in contact
        hipPos = rHip_feet(1:2);
        posCur = [-hipPos(1); 0; hipPos - rHip_feet(3:4)];
    else % right foot in contact
        hipPos = rHip_feet(3:4);
        posCur = [hipPos - rHip_feet(1:2); -hipPos(1); 0];
    end

    velCur = J(:, 1:4)*z(9:12); % foot velocity, relative to the hip (so ignore other terms)

    % get the errors to compute virtual forces
    posErr = posDes - posCur;
    velErr = velDes - velCur;

    f = accDes + K*posErr + D*velErr;   

    % Map to joint torques  
    Linv = J*(A\J');
    mu = L*J*(A\C) - (Linv\dJ)*dq;
    rho = (Linv\J)*(A\G);

    tau = J'*(Linv\f + mu + rho);
end

function qdot = discrete_impact_contact(z, p, sim)
    rest_coeff = sim.restitution_coeff;
    fric_coeff = sim.friction_coeff;
    yC = sim.ground_height;
    
    % Actual position and velocity of feet, wheel, and hip
    rFeet = position_feet(z, p);
    rllE = rFeet(:, 1);
    rrlE = rFeet(:, 2);
    rWheel = position_wheel(z, p);
    rHip = position_hip(z, p);

    drFeet = velocity_feet(z, p);
    drllE = drFeet(:, 1);
    drrlE = drFeet(:, 2);
    drWheel = velocity_wheel(z, p);
    drHip = velocity_hip(z, p);

    yl = rllE(2); yldot = drllE(2);
    yr = rrlE(2); yrdot = drrlE(2);
    yw = rWheel(2); ywdot = drWheel(2);
    yh = rHip(2); yhdot = drHip(2);

    qdot = z(9:16);

    % Compute the height of the feet, wheel, and hip relative to the ground
    C_yl = yl - yC;
    C_yr = yr - yC;
    C_yw = yw - yC;
    C_yh = yh - yC;

    % Compute the velocity of the feet relative to the ground
    C_yl_dot = yldot;
    C_yr_dot = yrdot;
    C_yw_dot = ywdot;
    C_yh_dot = yhdot;
    C_y_dot = [yldot; yrdot; ywdot; yhdot];

    % Choose which legs to model contact with (for debugging & comparison)
%     leg = 'left';
%     leg = 'right';
%     leg = 'both';
%     leg = 'both and wheel';
    leg = 'all';

    if strcmp(leg, 'left')
        % Check constraints
        if ((C_yl > 0 || C_yl_dot > 0))
            % If constraints aren't violated, don't update qdot
            return
        else
            % Compute vertical impulse force
            A = A_leggedWheelchair(z, p);
            J = jacobian_feet(z, p);
            J_c_yl = J(2, :); % Jacobian of left foot in y-directionn
            L_c_yl_inv = J_c_yl*(A\J_c_yl'); % operational space mass matrix in y-direction
    
            % Vertical forces on feet
            F_c_yl = L_c_yl_inv\(-rest_coeff*C_yl_dot - J_c_yl*qdot);
    
            % Update qdot
            qdot = qdot + A\(J_c_yl'*F_c_yl);
    
            % Compute tangential impulse forces
            J_c_xl = J(1, :); % Jacobian of left foot in x-direction
            L_c_xl_inv = J_c_xl*(A\J_c_xl');
            F_c_xl = L_c_xl_inv\(0 - J_c_xl*qdot);
    
            % Truncate F_c_x if it is outside of friction cone for each foot
            F_c_xl = max(min(F_c_xl, fric_coeff*F_c_yl), -fric_coeff*F_c_yl);
            % if F_c_xl > fric_coeff*F_c_yl
            %     F_c_xl = fric_coeff*F_c_yl;
            % elseif F_c_xl < -fric_coeff*F_c_yl
            %     F_c_xl = -fric_coeff*F_c_yl;
            % end
    
            % Update qdot
            qdot = qdot + A\(J_c_xl'*F_c_xl);
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
            C_y_dot = [yldot; yrdot; ywdot];
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

    elseif strcmp(leg, 'all')

         % Check constraints
        if ((C_yl > 0 || C_yl_dot > 0) && (C_yr > 0 || C_yr_dot > 0) && ...
                (C_yw > 0 || C_yw_dot > 0) && (C_yh > 0 || C_yh_dot > 0))
            % If constraints aren't violated, don't update qdot
            return
        else
            % Compute vertical impulse force
            A = A_leggedWheelchair(z,p);
            J  = jacobian_feet(z,p);
            J_c_y = vertcat(J(2,:), J(4,:), J(6,:), J(8,:));
            L_c_y = inv(J_c_y * inv(A) * J_c_y');
    
            % Vertical forces on feet
            F_c_y = L_c_y * (-rest_coeff * C_y_dot - J_c_y * qdot);
    
            % Update qdot
            qdot = qdot + inv(A) * (J_c_y'*F_c_y);
    
            % Compute tangential impulse forces
            J_c_x = vertcat(J(1,:), J(3,:), J(5,:), J(7,:));
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

            % hip
            if F_c_x(4) > fric_coeff * F_c_y(4)
                F_c_x(4) = fric_coeff * F_c_y(4);
            elseif F_c_x(4) < -fric_coeff * F_c_y(4)
                F_c_x(4) = -fric_coeff * F_c_y(4);
            end
    
            % Update qdot
            qdot = qdot + inv(A) * J_c_x' * F_c_x;
        end

    end
end

function qdot = joint_limit_constraint(z, p)
    % Joint limits for a given leg (visually estimated)
    qC1_min = deg2rad(-50);
    qC1_max = deg2rad(50);
    qC2_min = deg2rad(30);
    qC2_max = deg2rad(150);
    
    % Name variables for readability
    qdot = z(9:16);
    q1 = z(1);
    q2 = z(2);
    q3 = z(3);
    q4 = z(4);
    q1dot = z(9);
    q2dot = z(10);
    q3dot = z(11);
    q4dot = z(12);

    % Check joint constraints
    % Left leg (logic will look weird due to negative angle defitions)
    if (q1 > -qC1_min && q1dot < 0) || (q1 < -qC1_max && q1dot > 0) || (q1 < -qC1_min && q1 > -qC1_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(1) = 0;
    end

    if (q2 > -qC2_min && q2dot < 0) || (q2 < -qC2_max && q2dot > 0) || (q2 < -qC2_min && q2 > -qC2_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(2) = 0;
    end

    % Right leg
    if (q3 < qC1_min && q3dot > 0) || (q3 > qC1_max && q3dot < 0) || (q3 > qC1_min && q3 < qC1_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(3) = 0;
    end

    if (q4 < qC2_min && q4dot > 0) || (q4 > qC2_max && q4dot < 0) || (q4 > qC2_min && q4 < qC2_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(4) = 0;
    end
end