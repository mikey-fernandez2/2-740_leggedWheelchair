clear
name = 'leggedWheelchair';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 th2 th3 th4 th5 x y phi dth1 dth2 dth3 dth4 dth5 dx dy dphi ddth1 ddth2 ddth3 ddth4 ddth5 ddx ddy ddphi real
syms m1 m2 m5 ma mb I1 I2 I5 Ia Ib l_c1 l_c2 l_cb real
syms l1 l2 b r g real
syms tau1 tau2 tau3 tau4 Fx Fy real
syms Ir N real

% Group them
q   = [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi];      % generalized coordinates
dq  = [dth1 ; dth2 ; dth3 ; dth4 ; dth5 ; dx ; dy ; dphi];    % first time derivatives
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5; ddx; ddy; ddphi];  % second time derivatives
u   = [tau1 ; tau2 ; tau3 ; tau4];     % controls
F   = [Fx   ; Fy];

p   = [m1 m2 m5 ma mb I1 I2 I5 Ia Ib l1 l2 b l_c1 l_c2 l_cb r Ir N g]';        % parameters

% Generate Vectors and Derivativess
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat, jhat);

xhat = [1; 0; 0];
yhat = [0; 1; 0];

ebhat = cos(th5)*ihat + sin(th5)*jhat;
el1hat = sin(th1 + th5)*ihat - cos(th1 + th5)*jhat;
el2hat = sin(th1 + th2 + th5)*ihat - cos(th1 + th2 + th5)*jhat;
er1hat = sin(th3 + th5)*ihat - cos(th3 + th5)*jhat;
er2hat = sin(th3 + th4 + th5)*ihat - cos(th3 + th4 + th5)*jhat;

ddt = @(r) jacobian(r, [q; dq])*[dq; ddq]; % a handy anonymous function for taking time derivatives

rA = x*ihat + y*jhat;
rB = rA + b*ebhat;
rC = rB + l1*el1hat;
rD = rB + l1*er1hat;
rE = rC + l2*el2hat;
rF = rD + l2*er2hat;

r_lc1 = rB + l_c1*el1hat;
r_lc2 = rC + l_c2*el2hat;
r_rc1 = rB + l_c1*er1hat;
r_rc2 = rD + l_c2*er2hat;
r_cb = rA + l_cb*ebhat;

drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);
drF = ddt(rF);

dr_lc1 = ddt(r_lc1);
dr_lc2 = ddt(r_lc2);
dr_rc1 = ddt(r_rc1);
dr_rc2 = ddt(r_rc2);
dr_cb = ddt(r_cb);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F, r) simplify(jacobian(r, q)'*(F));    % force contributions to generalized forces
M2Q = @(M, w) simplify(jacobian(w, dq)'*(M));   % moment contributions to generalized forces

omega1 = dth5 + dth1;
omega2 = dth5 + dth1 + dth2;
omega3 = dth5 + dth1 + dth2;
omega4 = dth5 + dth1;

T1 = (1/2)*m1*dot(dr_m1, dr_m1) + (1/2)*I1*omega1^2;
T2 = (1/2)*m2*dot(dr_m2, dr_m2) + (1/2)*I2*omega2^2;
T3 = (1/2)*m3*dot(dr_m3, dr_m3) + (1/2)*I3*omega3^2;
T4 = (1/2)*m4*dot(dr_m4, dr_m4) + (1/2)*I4*omega4^2;
T1r = (1/2)*Ir*(N*dth1)^2;
T2r = (1/2)*Ir*(dth1 + N*dth2)^2;

Vg1 = m1*g*dot(r_m1, -ihat);
Vg2 = m2*g*dot(r_m2, -ihat);
Vg3 = m3*g*dot(r_m3, -ihat);
Vg4 = m4*g*dot(r_m4, -ihat);

T = simplify(T1 + T2 + T3 + T4 + T1r + T2r);
Vg = Vg1 + Vg2 + Vg3 + Vg4;
Q_tau1 = M2Q(tau1*khat, omega1*khat);
Q_tau2 = M2Q(tau2*khat, omega2*khat); 
Q_tau2R = M2Q(-tau2*khat, omega1*khat);

Q_tau = Q_tau1 + Q_tau2 + Q_tau2R;

Q = Q_tau;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T + Vg;
L = T - Vg;
eom = ddt(jacobian(L, dq).') - jacobian(L, q).' - Q;

% Rearrange Equations of Motion
A = simplify(jacobian(eom, ddq));
b = A*ddq - eom;

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A;
Grav_Joint_Sp = simplify(jacobian(Vg, q)');
Corr_Joint_Sp = simplify(eom + Q - Grav_Joint_Sp - A*ddq);

% Compute foot jacobian
J = jacobian(rE, q);

% Compute ddt( J )
dJ= reshape(ddt(J(:)), size(J));

% Write Energy Function and Equations of Motion
z  = [q; dq];

rE = rE(1:2);
drE= drE(1:2);
J  = J(1:2, 1:2);
dJ = dJ(1:2, 1:2);

matlabFunction(A,'file', ['A_' name], 'vars', {z p});
matlabFunction(b,'file', ['b_' name], 'vars', {z u p});
matlabFunction(E,'file', ['energy_' name], 'vars', {z p});
matlabFunction(rE,'file', 'position_foot', 'vars', {z p});
matlabFunction(drE,'file', 'velocity_foot', 'vars', {z p});
matlabFunction(J ,'file', 'jacobian_foot', 'vars', {z p});
matlabFunction(dJ ,'file', 'jacobian_dot_foot', 'vars', {z p});

matlabFunction(Grav_Joint_Sp, 'file', 'Grav_leg', 'vars', {z p});
matlabFunction(Corr_Joint_Sp, 'file', 'Corr_leg', 'vars', {z p});
matlabFunction(keypoints, 'file', ['keypoints_' name], 'vars', {z p});