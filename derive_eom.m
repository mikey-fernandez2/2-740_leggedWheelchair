clear; clc;
name = 'leggedWheelchair';

%% Define the system
% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 th2 th3 th4 th5 x y phi dth1 dth2 dth3 dth4 dth5 dx dy dphi ddth1 ddth2 ddth3 ddth4 ddth5 ddx ddy ddphi real
syms m1 m2 ma mb I_1 I_2 I_5 I_A I_B l_c1 l_c2 l_cb real
syms l1 l2 b r g real
syms tau1 tau2 tau3 tau4 Fl_x Fl_y Fr_x Fr_y Fw_x Fw_y real
syms Ir N real
syms mU I_U lU phiU real % constant offset to the user with mass mU

% Group them
q   = [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];  % generalized coordinates
dq  = [dth1 ; dth2 ; dth3 ; dth4 ; dth5 ; dx ; dy ; dphi ];  % first time derivatives
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5; ddx; ddy; ddphi];  % second time derivatives
u   = [tau1 ; tau2 ; tau3 ; tau4];                    % controls
F   = [Fl_x ; Fl_y ; Fr_x ; Fr_y ; Fw_x ; Fw_y];      % ground reaction forces

p   = [lU phiU mU I_U m1 m2 ma mb I_1 I_2 I_5 I_A I_B l1 l2 b l_c1 l_c2 l_cb r Ir N g]';        % parameters

%% Kinematics
% Generate Vectors and Derivativess
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat, jhat);

xhat = [1; 0; 0];
yhat = [0; 1; 0];

ebhat =  cos(th5)*ihat             + sin(th5)*jhat; % orientation of connecting rod
el1hat = sin(th1 + th5)*ihat       - cos(th1 + th5)*jhat; % orientation of link 1 of left leg
el2hat = sin(th1 + th2 + th5)*ihat - cos(th1 + th2 + th5)*jhat; % orientation of link 2 of left leg
er1hat = sin(th3 + th5)*ihat       - cos(th3 + th5)*jhat; % orientation of link 1 of right leg
er2hat = sin(th3 + th4 + th5)*ihat - cos(th3 + th4 + th5)*jhat; % orientation of link 2 of right leg
eUhat =  cos(th5 + phiU)*ihat      + sin(th5 + phiU)*jhat; % orientation of vector from wheel axis to user 

ddt = @(r) jacobian(r, [q; dq])*[dq; ddq]; % a handy anonymous function for taking time derivatives

rA = x*ihat + y*jhat; % center of wheel
rB = rA +  b*ebhat;  % robot hip/shoulder
rU = rA + lU*eUhat;  % CoM of user of chair
rC = rB + l1*el1hat; % left knee
rD = rB + l1*er1hat; % right knee
rE = rC + l2*el2hat; % left foot
rF = rD + l2*er2hat; % right foot

r_lc1 = rB + l_c1*el1hat; % CoM of left link 1
r_lc2 = rC + l_c2*el2hat; % CoM of left link 2
r_rc1 = rB + l_c1*er1hat; % CoM of right link 1
r_rc2 = rD + l_c2*er2hat; % CoM of right link 2
r_cb = rA + l_cb*ebhat; % CoM of connecting rod

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);
drF = ddt(rF);
drU = ddt(rU);

dr_lc1 = ddt(r_lc1);
dr_lc2 = ddt(r_lc2);
dr_rc1 = ddt(r_rc1);
dr_rc2 = ddt(r_rc2);
dr_cb = ddt(r_cb);

%% Kinetics
% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F, r) simplify(jacobian(r, q)'*(F));    % force contributions to generalized forces
M2Q = @(M, w) simplify(jacobian(w, dq)'*(M));   % moment contributions to generalized forces

% angular velocities
omega_l1 = dth5 + dth1;
omega_l2 = dth5 + dth1 + dth2;
omega_r1 = dth5 + dth3;
omega_r2 = dth5 + dth3 + dth4;
omega_U =  dth5; % constant offset, so the angular velocity matches the connecting rod tilt

% kinetic energies
T_A  = (1/2)*ma*dot(drA, drA)       + (1/2)*I_A*dphi^2;
T_B  = (1/2)*mb*dot(dr_cb, dr_cb)   + (1/2)*I_B*dth5^2;
T_U  = (1/2)*mU*dot(drU, drU)       + (1/2)*I_U*omega_U^2;
T_l1 = (1/2)*m1*dot(dr_lc1, dr_lc1) + (1/2)*I_1*omega_l1^2;
T_l2 = (1/2)*m2*dot(dr_lc2, dr_lc2) + (1/2)*I_2*omega_l2^2;
T_r1 = (1/2)*m1*dot(dr_rc1, dr_rc1) + (1/2)*I_1*omega_r1^2;
T_r2 = (1/2)*m2*dot(dr_rc2, dr_rc2) + (1/2)*I_2*omega_r2^2;

T_l1_rotor = (1/2)*Ir*(dth5 + N*dth1)^2;
T_l2_rotor = (1/2)*Ir*(dth5 + dth1 + N*dth2)^2;
T_r1_rotor = (1/2)*Ir*(dth5 + N*dth3)^2;
T_r2_rotor = (1/2)*Ir*(dth5 + dth3 + N*dth4)^2;

Vg_A = ma*g*dot(rA, -jhat);
Vg_B = mb*g*dot(r_cb, -jhat);
Vg_U = mU*g*dot(rU, -jhat);
Vg_l1 = m1*g*dot(r_lc1, -jhat);
Vg_l2 = m2*g*dot(r_lc2, -jhat);
Vg_r1 = m1*g*dot(r_rc1, -jhat);
Vg_r2 = m2*g*dot(r_rc2, -jhat);

T = simplify(T_A + T_B + T_U + T_l1 + T_l2 + T_r1 + T_r2 + T_l1_rotor + T_l2_rotor + T_r1_rotor + T_r2_rotor);
Vg = simplify(Vg_A + Vg_B + Vg_U + Vg_l1 + Vg_l2 + Vg_r1 + Vg_r2);

Q_tau1 = M2Q(tau1*khat, omega_l1*khat);
Q_tau2 = M2Q(tau2*khat, omega_l2*khat); 
Q_tau2R = M2Q(-tau2*khat, omega_l1*khat);
Q_tau3 = M2Q(tau3*khat, omega_r1*khat);
Q_tau4 = M2Q(tau4*khat, omega_r2*khat); 
Q_tau4R = M2Q(-tau4*khat, omega_r1*khat);
% motor torques don't impact th5, that's a driven variable
% Q_tau1 = M2Q(tau1*khat, (omega_l1 - dth5)*khat);
% Q_tau2 = M2Q(tau2*khat, (omega_l2 - dth5)*khat); 
% Q_tau2R = M2Q(-tau2*khat, (omega_l1 - dth5)*khat);
% Q_tau3 = M2Q(tau3*khat, (omega_r1 - dth5)*khat);
% Q_tau4 = M2Q(tau4*khat, (omega_r2 - dth5)*khat); 
% Q_tau4R = M2Q(-tau4*khat, (omega_r1 - dth5)*khat);

Q_tau = Q_tau1 + Q_tau2 + Q_tau2R + Q_tau3 + Q_tau4 + Q_tau4R;
Q = Q_tau;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) rF(1:2) rU(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T + Vg;
L = T - Vg;
eom = simplify(ddt(jacobian(L, dq).') - jacobian(L, q).' - Q);

% Rearrange Equations of Motion
A = simplify(jacobian(eom, ddq));
b = simplify(A*ddq - eom);

% Equations of motion are
% eom = A*ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A;
Grav_Joint_Sp = simplify(jacobian(Vg, q)');
Corr_Joint_Sp = simplify(eom + Q - Grav_Joint_Sp - A*ddq);

% Compute feet jacobian
J = jacobian([rE; rF], q);

% Compute ddt( J )
dJ = reshape(ddt(J(:)), size(J));

% Write Energy Function and Equations of Motion
z  = [q; dq];

rFeet = [rE(1:2) rF(1:2)];
drFeet = [drE(1:2) drF(1:2)];
J  = J([1 2 4 5], :);
dJ = dJ([1 2 4 5], :);

matlabFunction(A, 'file', ['Derivation/A_' name], 'vars', {z p});
matlabFunction(b, 'file', ['Derivation/b_' name], 'vars', {z u p});
matlabFunction(E, 'file', ['Derivation/energy_' name], 'vars', {z p});
matlabFunction(rFeet, 'file', 'Derivation/position_feet', 'vars', {z p});
matlabFunction(drFeet, 'file', 'Derivation/velocity_feet', 'vars', {z p});
matlabFunction(rU, 'file', 'Derivation/position_user', 'vars', {z p});
matlabFunction(drU, 'file', 'Derivation/velocity_user', 'vars', {z p});
matlabFunction(J, 'file', 'Derivation/jacobian_feet', 'vars', {z p});
matlabFunction(dJ, 'file', 'Derivation/jacobian_dot_feet', 'vars', {z p});

matlabFunction(Grav_Joint_Sp, 'file', 'Derivation/Grav_leg', 'vars', {z p});
matlabFunction(Corr_Joint_Sp, 'file', 'Derivation/Corr_leg', 'vars', {z p});
matlabFunction(keypoints, 'file', ['Derivation/keypoints_' name], 'vars', {z p});