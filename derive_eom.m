clear; clc;
name = 'leggedWheelchair';

%% Define the system
% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 th2 th3 th4 th5 x y phi dth1 dth2 dth3 dth4 dth5 dx dy dphi ddth1 ddth2 ddth3 ddth4 ddth5 ddx ddy ddphi real
syms m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B real
syms l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r g real
syms tau1 tau2 tau3 tau4 Fl_x Fl_y Fr_x Fr_y Fw_x Fw_y real
syms Ir N real
syms mU I_U lU phiU real % constant offset to the user with mass mU

% Group them
q   = [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];  % generalized coordinates
dq  = [dth1 ; dth2 ; dth3 ; dth4 ; dth5 ; dx ; dy ; dphi ];  % first time derivatives
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5; ddx; ddy; ddphi];  % second time derivatives
u   = [tau1 ; tau2 ; tau3 ; tau4];                    % controls
F   = [Fl_x ; Fl_y ; Fr_x ; Fr_y ; Fw_x ; Fw_y];      % ground reaction forces

p = [m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';
% p   = [lU phiU mU I_U m1 m2 ma mb I1 I2 I_A I_B l1 l2 b l_c1 l_c2 l_cb r Ir N g]';        % parameters

%% Kinematics
% Generate Vectors and Derivativess
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat, jhat);

xhat = [1; 0; 0];
yhat = [0; 1; 0];

% ebhat =  cos(th5)*ihat             + sin(th5)*jhat;             % orientation of connecting rod
% el1hat = sin(-th5 - th1)*ihat       - cos(-th5 - th1)*jhat;       % orientation of link 1/4 of left leg - see system diagram for sign convention
% el2hat = sin(-th5 - th1 - th2)*ihat - cos(-th5 - th1 - th2)*jhat; % orientation of link 2/3 of left leg
% er1hat = sin(th3 + th5)*ihat       - cos(th3 + th5)*jhat;       % orientation of link 1/4 of right leg
% er2hat = sin(th3 + th4 + th5)*ihat - cos(th3 + th4 + th5)*jhat; % orientation of link 2/3 of right leg
% eUhat =  cos(th5 + phiU)*ihat      + sin(th5 + phiU)*jhat;      % orientation of vector from wheel axis to user 
ebhat =  cos(th5)            *ihat + sin(th5)            *jhat; % orientation of connecting rod
el1hat = sin(th5 - th1)      *ihat - cos(th5 - th1)      *jhat; % orientation of link 1/4 of left leg - see system diagram for sign convention
el2hat = sin(th5 - th1 - th2)*ihat - cos(th5 - th1 - th2)*jhat; % orientation of link 2/3 of left leg
er1hat = sin(th5 + th3)      *ihat - cos(th5 + th3)      *jhat; % orientation of link 1/4 of right leg
er2hat = sin(th5 + th3 + th4)*ihat - cos(th5 + th3 + th4)*jhat; % orientation of link 2/3 of right leg
eUhat =  cos(th5 + phiU)     *ihat + sin(th5 + phiU)     *jhat; % orientation of vector from wheel axis to user 

ddt = @(r) jacobian(r, [q; dq])*[dq; ddq]; % a handy anonymous function for taking time derivatives

rA = x*ihat + y*jhat;   % center of wheel
rB = rA +  b*ebhat;     % robot hip/shoulder
rU = rA + lU*eUhat;     % CoM of user of chair

rllA = rB + l_OA*el1hat;
rllB = rB + l_OB*el1hat; % left knee
rllC = rllA + l_AC*el2hat;
rllD = rllB + l_AC*el2hat;
rllE = rllD + l_DE*el1hat; % left foot
r_lc1 = rB + l_O_m1*el1hat;
r_lc2 = rllB + l_B_m2*el2hat;
r_lc3 = rllA + l_A_m3*el2hat;
r_lc4 = rllC + l_C_m4*el1hat;

rrlA = rB + l_OA*er1hat;
rrlB = rB + l_OB*er1hat; % right knee
rrlC = rrlA + l_AC*er2hat;
rrlD = rrlB + l_AC*er2hat;
rrlE = rrlD + l_DE*er1hat; % right foot
r_rc1 = rB + l_O_m1*er1hat;
r_rc2 = rllB + l_B_m2*er2hat;
r_rc3 = rrlA + l_A_m3*er2hat;
r_rc4 = rrlC + l_C_m4*er1hat;

r_cb = rA + l_cb*ebhat; % CoM of connecting rod
rW = rA - r*jhat; % lowest point of wheel

drllA = ddt(rllA);
drllB = ddt(rllB);
drllC = ddt(rllC);
drllD = ddt(rllD);
drllE = ddt(rllE);
drl_c1 = ddt(r_lc1);
drl_c2 = ddt(r_lc2);
drl_c3 = ddt(r_lc3);
drl_c4 = ddt(r_lc4);

drrlA = ddt(rrlA);
drrlB = ddt(rrlB);
drrlC = ddt(rrlC);
drrlD = ddt(rrlD);
drrlE = ddt(rrlE);
drr_c1 = ddt(r_rc1);
drr_c2 = ddt(r_rc2);
drr_c3 = ddt(r_rc3);
drr_c4 = ddt(r_rc4);

drA = ddt(rA);
drB = ddt(rB);
drU = ddt(rU);
dr_cb = ddt(r_cb);
drW = ddt(rW);

% get the user's acceleration as well
ddrU = ddt(drU);
userAccel = [ddrU(1:2); ddth5];

%% Kinetics
% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F, r) simplify(jacobian(r, q)'*(F));    % force contributions to generalized forces
M2Q = @(M, w) simplify(jacobian(w, dq)'*(M));   % moment contributions to generalized forces

% angular velocities
omega_l1 = dth5 - dth1; % left links 1/4
omega_l2 = dth5 - dth1 - dth2; % left links 2/3
omega_r1 = dth5 + dth3; % right links 1/4
omega_r2 = dth5 + dth3 + dth4; % right links 2/3
omega_U =  dth5; % constant offset, so the angular velocity matches the connecting rod tilt

% kinetic energies
T_A  = (1/2)*ma*dot(drA, drA)       + (1/2)*I_A*dphi^2;
T_B  = (1/2)*mb*dot(dr_cb, dr_cb)   + (1/2)*I_B*dth5^2;
T_U  = (1/2)*mU*dot(drU, drU)       + (1/2)*I_U*omega_U^2;
T_l1 = (1/2)*m1*dot(drl_c1, drl_c1) + (1/2)*I1*omega_l1^2;
T_l2 = (1/2)*m2*dot(drl_c2, drl_c2) + (1/2)*I2*omega_l2^2;
T_l3 = (1/2)*m3*dot(drl_c3, drl_c3) + (1/2)*I3*omega_l2^2;
T_l4 = (1/2)*m4*dot(drl_c4, drl_c4) + (1/2)*I4*omega_l1^2;
T_r1 = (1/2)*m1*dot(drr_c1, drr_c1) + (1/2)*I1*omega_r1^2;
T_r2 = (1/2)*m2*dot(drr_c2, drr_c2) + (1/2)*I2*omega_r2^2;
T_r3 = (1/2)*m3*dot(drr_c3, drr_c3) + (1/2)*I3*omega_r2^2;
T_r4 = (1/2)*m4*dot(drr_c4, drr_c4) + (1/2)*I4*omega_r1^2;

T_l1_rotor = (1/2)*Ir*(dth5 - N*dth1)^2;
T_l2_rotor = (1/2)*Ir*(dth5 - dth1 - N*dth2)^2;
T_r1_rotor = (1/2)*Ir*(dth5 + N*dth3)^2;
T_r2_rotor = (1/2)*Ir*(dth5 + dth3 + N*dth4)^2;

Vg_A  = ma*g*dot(rA, jhat);
Vg_B  = mb*g*dot(r_cb, jhat);
Vg_U  = mU*g*dot(rU, jhat);
Vg_l1 = m1*g*dot(r_lc1, jhat);
Vg_l2 = m2*g*dot(r_lc2, jhat);
Vg_l3 = m3*g*dot(r_lc3, jhat);
Vg_l4 = m4*g*dot(r_lc4, jhat);
Vg_r1 = m1*g*dot(r_rc1, jhat);
Vg_r2 = m2*g*dot(r_rc2, jhat);
Vg_r3 = m3*g*dot(r_rc3, jhat);
Vg_r4 = m4*g*dot(r_rc4, jhat);

T = simplify(T_A + T_B + T_U + T_l1 + T_l2 + T_l3 + T_l4 + T_r1 + T_r2 + ...
    T_r3 + T_r4 + T_l1_rotor + T_l2_rotor + T_r1_rotor + T_r2_rotor);
Vg = simplify(Vg_A + Vg_B + Vg_U + Vg_l1 + Vg_l2 + Vg_l3 + Vg_l4 + Vg_r1 + ...
    Vg_r2 + Vg_r3 + Vg_r4);

% motor torques
Q_tau1 = M2Q(-tau1*khat, omega_l1*khat); % should the sign for tau1 and tau2 be flipped?
Q_tau2 = M2Q(-tau2*khat, omega_l2*khat); 
Q_tau2R = M2Q(tau2*khat, omega_l1*khat);
Q_tau3 = M2Q(tau3*khat, omega_r1*khat);
Q_tau4 = M2Q(tau4*khat, omega_r2*khat); 
Q_tau4R = M2Q(-tau4*khat, omega_r1*khat);

Q_tau = Q_tau1 + Q_tau2 + Q_tau2R + Q_tau3 + Q_tau4 + Q_tau4R;
Q = Q_tau;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rllA(1:2) rllB(1:2) rllC(1:2) rllD(1:2) rllE(1:2)...
    rrlA(1:2) rrlB(1:2) rrlC(1:2) rrlD(1:2) rrlE(1:2) rU(1:2) rW(1:2)];

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
rFeet = [rllE(1:2) rrlE(1:2)];
rWheel = rW(1:2);
rHip = rB(1:2);
drFeet = [drllE(1:2) drrlE(1:2)];
drWheel = drW(1:2);
drHip = drB(1:2);
J = jacobian([rFeet(:,1); rFeet(:,2); rWheel; rHip], q);

% Compute ddt( J )
dJ = reshape(ddt(J(:)), size(J));

% Write Energy Function and Equations of Motion
z  = [q; dq];
dz = [dq; ddq];

matlabFunction(A, 'file', ['Derivation/A_' name], 'vars', {z p});
matlabFunction(b, 'file', ['Derivation/b_' name], 'vars', {z u p});
matlabFunction(E, 'file', ['Derivation/energy_' name], 'vars', {z p});
matlabFunction(rFeet, 'file', 'Derivation/position_feet', 'vars', {z p});
matlabFunction(drFeet, 'file', 'Derivation/velocity_feet', 'vars', {z p});
matlabFunction(rU, 'file', 'Derivation/position_user', 'vars', {z p});
matlabFunction(drU, 'file', 'Derivation/velocity_user', 'vars', {z p});
matlabFunction(userAccel, 'file', 'Derivation/acceleration_user', 'vars', {z, dz, p});
matlabFunction(rWheel, 'file', 'Derivation/position_wheel', 'vars', {z p});
matlabFunction(drWheel, 'file', 'Derivation/velocity_wheel', 'vars', {z p});
matlabFunction(rHip, 'file', 'Derivation/position_hip', 'vars', {z p});
matlabFunction(drHip, 'file', 'Derivation/velocity_hip', 'vars', {z p});
matlabFunction(J, 'file', 'Derivation/jacobian_feet', 'vars', {z p});
matlabFunction(dJ, 'file', 'Derivation/jacobian_dot_feet', 'vars', {z p});

matlabFunction(Grav_Joint_Sp, 'file', 'Derivation/Grav_leg', 'vars', {z p});
matlabFunction(Corr_Joint_Sp, 'file', 'Derivation/Corr_leg', 'vars', {z p});
matlabFunction(keypoints, 'file', ['Derivation/keypoints_' name], 'vars', {z p});