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
        qdot_plus = joint_limit_constraint(z_out(:,i), p); % is this potentially a problem with joint limits overriding contact?

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
    [kinematicsDes, ~] = ctrlStruct.gait.footPatternGenerator(t);
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

    % get the errors to compute virtual forces
    posErr = posDes - posCur;
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

    constraintsnotViolated = [C_yl > 0 || C_yl_dot > 0;
                              C_yr > 0 || C_yr_dot > 0;
                              C_yw > 0 || C_yw_dot > 0;
                              C_yh > 0 || C_yh_dot > 0];
    constraintsViolated = [C_yl < 0 && C_yl_dot < 0;
                           C_yr < 0 && C_yr_dot < 0;
                           C_yw < 0 && C_yw_dot < 0;
                           C_yh < 0 && C_yh_dot < 0];

     % Check constraints
    if (all(constraintsnotViolated))
        % If constraints aren't violated, don't update qdot
        return
    else

        F_thresh = 0.01; % convergence force threshold
        converged = false;
        iter = 0;

        rowsY = [2 4 6 8]; rowsX = [1 3 5 7];

        while converged == false

            for contact = 1:length(constraintsViolated) % loop through all potential contact points
    
                if constraintsViolated(contact) % only update force for points that are in contact

                    % Create selection array of 0s with a 1 at index of current contact
                    contactArray = zeros(length(constraintsViolated), 1);
                    contactArray(contact) = 1;
        
                    % Compute vertical impulse force on contact
                    A = A_leggedWheelchair(z, p);
                    J = jacobian_feet(z, p);
                    J_c_y = J(rowsY*contactArray, :); % just select row of Jacobian corresponding to current contact
                    L_c_y_inv = J_c_y*(A\J_c_y');
                    F_c_y = L_c_y_inv\(-rest_coeff*C_y_dot'*contactArray - J_c_y*qdot); % re-calculate C_y_dot at each contact point?
        
                    % Update qdot with vertical reaction forces
                    qdot = qdot + (A\(J_c_y'*F_c_y));
                    vcz = J_c_y*qdot; % compute vertical velocity of contact after updating qdot
            
                    % Compute tangential impulse forces
                    J_c_x = J(rowsX(contact), :);
                    L_c_x_inv = J_c_x*(A\J_c_x');
                    F_c_x = zeros(4, 1);
                    F_c_x(contact) = L_c_x_inv\(0 - J_c_x*qdot);
        
                    % Before computing friction forces, set vertical forces of
                    % points not in contact to 0
                    temp = F_c_y;
                    F_c_y = zeros(4, 1); 
                    F_c_y(contact) = temp;
            
                    % Truncate F_c_x if it is outside of friction cone for each point of contact
                    % left foot
                    if F_c_x(1) > fric_coeff*F_c_y(1)
                        F_c_x(1) = fric_coeff*F_c_y(1);
                    elseif F_c_x(1) < -fric_coeff*F_c_y(1)
                        F_c_x(1) = -fric_coeff*F_c_y(1);
                    end
            
                    % right foot
                    if F_c_x(2) > fric_coeff*F_c_y(2)
                        F_c_x(2) = fric_coeff*F_c_y(2);
                    elseif F_c_x(2) < -fric_coeff*F_c_y(2)
                        F_c_x(2) = -fric_coeff*F_c_y(2);
                    end
            
                    % wheel
                    if F_c_x(3) > wheel_fric*F_c_y(3)
                        F_c_x(3) = wheel_fric*F_c_y(3);
                    elseif F_c_x(3) < -wheel_fric*F_c_y(3)
                        F_c_x(3) = -wheel_fric*F_c_y(3);
                    end
            
                    % hip
                    if F_c_x(4) > fric_coeff*F_c_y(4)
                        F_c_x(4) = fric_coeff*F_c_y(4);
                    elseif F_c_x(4) < -fric_coeff*F_c_y(4)
                        F_c_x(4) = -fric_coeff*F_c_y(4);
                    end
            
                    % Update qdot
                    F_c_x = F_c_x(contact);
                    qdot = qdot + (A\J_c_x'*F_c_x);
                    z = [z(1:8); qdot];

                end
            end

            iter = iter + 1;

            if iter == 1 % don't do multiple iterations
                converged = true;
            end
                    
            if all(vcz >= 0) && all(temp >= 0) && all(abs(vcz.*temp) < F_thresh)
                converged = true;
                disp(strcat('Iterations: ', num2str(iter)))
                disp(strcat('vcz: ', num2str(vcz)))
                disp(strcat('F_c_y: ', num2str(temp)))
            elseif iter > 10
                converged = true;
                disp('MAX iterations reached!')
                disp(strcat('Iterations: ', num2str(iter)))
                disp(strcat('vcz: ', num2str(vcz)))
                disp(strcat('F_c_y: ', num2str(temp)))
            end

        end

        disp('~~ looped thru all contacts ~~')
        qdot

    end
end

function qdot = joint_limit_constraint(z, p)
    % Joint limits for a given leg (visually estimated)
    qC1_min = deg2rad(-135);
    qC1_max = deg2rad(0);
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
        disp('q1 JOINT LIMIT HIT')
    end

    if (q2 > -qC2_min && q2dot < 0) || (q2 < -qC2_max && q2dot > 0) || (q2 < -qC2_min && q2 > -qC2_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(2) = 0;
        disp('q2 JOINT LIMIT HIT')
    end

    % Right leg
    if (q3 < qC1_min && q3dot > 0) || (q3 > qC1_max && q3dot < 0) || (q3 > qC1_min && q3 < qC1_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(3) = 0;
        disp('q3 JOINT LIMIT HIT')
    end

    if (q4 < qC2_min && q4dot > 0) || (q4 > qC2_max && q4dot < 0) || (q4 > qC2_min && q4 < qC2_max)
        % If constraints aren't violated, or joint is moving out of
        % violation, don't update qdot
    else
        qdot(4) = 0;
        disp('q4 JOINT LIMIT HIT')
    end
end