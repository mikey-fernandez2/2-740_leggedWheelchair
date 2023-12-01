%% 2.740 Team 1 Legged Wheelchair Testing Script
% This script runs a single simulation with hand-picked parameters.
% It uses the same simulation procedure as main.m, but does not
% do a grid parameter search and outputs visualizations of the results.

% clc; clear; close all;

%%
setpath  % add subfolders

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
m_axle = 0.25*pi*(.25*.0254)^2 * 5*.0254 * 2710; % aluminum
ma = 2*0.3175 + m_axle; % 317.5 grams per wheel + axle
mb = m5 + 0.5 + 0.5; % total guess; mass of motors plus mounting hardware plus electronics box
b = 0.13; % diagonal length from wheel axle to hip

r = 0.05; % wheels from Amazon

l_OA = 0.011; % same as l_OA from lab (not quite accurate)
l_OB = 0.042; % same as l_OB from lab (not quite accurate)
l_AC = 0.096; % same as l_AC from lab (not quite accurate)
l_DE = 0.091; % same as l_DE from lab (not quite accurate)
l_O_m1 = 0.032; % l_O_m1
l_B_m2 = 0.0344; % l_B_m2
l_A_m3 = 0.0622; % l_A_m3 from lab
l_C_m4 = 0.0610; % l_C_m4 from lab (revisit this)
l_cb = 0.0675; % vertical distance to approximate CoM of connecting rod assembly

I1 = 25.1 * 10^-6;
I2 = 53.5 * 10^-6;
I3 = 9.25 * 10^-6;
I4 = 22.176 * 10^-6;
I_A = 0.5 * ma * r^2; % thin solid disk
I_B = 1/12 * m5 * ((.0254)^2 + (12*.0254)^2) + .450 * 0.11^2; % inertia of a rectangle plus point inertia for motors

N = 18.75;
Ir = 0.0035/N^2;
g = 9.81;

% Ground contact properties
restitution_coeff = 0.1;
friction_coeff = 0.8;
ground_height = 0;

% Parameter vector
p = [m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';

%% Simulation parameters
sim = struct();

sim.dt = 0.005;
sim.tf = 5.0;
sim.num_steps = floor(sim.tf/sim.dt);
sim.tspan = linspace(0, sim.tf, sim.num_steps);

% Ground contact properties
sim.restitution_coeff = restitution_coeff;
sim.friction_coeff = friction_coeff;
sim.wheel_friction = 0.01*friction_coeff;
sim.ground_height = ground_height;

% jointConstraints
sim.qC1_min = deg2rad(-135);
sim.qC1_max = deg2rad(0);
sim.qC2_min = deg2rad(30);
sim.qC2_max = deg2rad(150);

% order of generalized coordinates [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];
% sim.z0 = [pi/24; -pi/4; -pi/24; pi/4; pi/4; 0.5; 0.06; 0; 0; 0; 0; 0; 0; 0; 0; 0];
sim.z0 = [deg2rad(35); deg2rad(-90); deg2rad(-35); deg2rad(90); deg2rad(40); 0; 0.075; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%% Gait parameters
% CHOOSE A SINGLE SET OF PARAMETERS HERE instead of looping through parameters %
tStance = 0.5;
gdPen = 0.05;
avgVel = 0.1;
K = 300;
D = 30;

tSwing = 0.5; % seconds
nomHip = [-0.1/(tStance*avgVel) + 1; 0.125]; % nominal hip position
ctrlPts = [0.00 0.100 0.500 0.900 1.00;
           0.00 0.075 0.050 0.075 0.00]; % control points for Bezier trajectory, normalized in [0, 1]

%% Simulate
storedVals = {'xSmooth', 'pitchSmooth', 'tStance', 'gdPen', 'avgVel', 'K', 'D'};
resultsTable = cell2table(cell(0, length(storedVals)), 'VariableNames', storedVals);

ctrlStruct = struct();

fprintf('tStance = %4.3f | gdPen = %4.3f | avgVel = %4.3f | K = %4.3f | D = %4.3f\n', [tStance, gdPen, avgVel, K, D])

% get the settings for this run
gaitGen = GaitGenerator(ctrlPts, nomHip, tStance, tSwing, gdPen, avgVel);
ctrlStruct.gait = gaitGen;
ctrlStruct.K = K;
ctrlStruct.D = D;

% simulate with these settings
[z_out, dz_out] = simulate(sim, p, ctrlStruct);

% calculate and store the smoothness values
% get the acceleration of the user
accelUser = acceleration_user(z_out, dz_out, p);
xAccel = accelUser(1, :); pitchAccel = accelUser(3, :);
% calculate smoothness
xSmooth = mean(xAccel.^2); pitchSmooth = mean(pitchAccel.^2);

% store results in the table
theseResults = {xSmooth, pitchSmooth, tStance, gdPen, avgVel, K, D};
resultsTable = [resultsTable; theseResults];

%% Compute foot position and velocity over time
dt = sim.dt; num_steps = sim.num_steps; tspan = sim.tspan;
rFeet = zeros(2,2,length(tspan));
vFeet = zeros(2,2,length(tspan));
footTraj = zeros(12, length(tspan));
rHip = zeros(2,length(tspan));
for i = 1:length(tspan)
    rFeet(:,:,i) = position_feet(z_out(:,i),p);
    vFeet(:,:,i) = velocity_feet(z_out(:,i),p);
    [footTraj(:,i), ~] = gaitGen.footPatternGenerator(i*dt);
    rHip(:, i) = position_hip(z_out(:,i),p);
end

%% Plot & animate results
% Plot position of feet over time
figure(1)
plot(tspan,squeeze(rFeet(1,1,:)),'LineWidth',2) % left, x
hold on
plot(tspan,squeeze(rFeet(2,1,:)),'LineWidth',2) % left, y
plot(tspan,squeeze(rFeet(1,2,:)),'LineWidth',2) % right, x
plot(tspan,squeeze(rFeet(2,2,:)),'LineWidth',2) % right, y
plot(tspan, footTraj(1:4,:));               % desired feet trajectories relative to hip position

xlabel('Time (s)'); ylabel('Position (m)'); legend({'L_x','L_y','R_x','R_y',...
    'L_{xdes}','L_{ydes}','R_{xdes}','R_{ydes}'});
title('Feet Position')

% Plot velocity of feet over time
figure(2)
plot(tspan,squeeze(vFeet(1,1,:)),'LineWidth',2) % left, x
hold on
plot(tspan,squeeze(vFeet(2,1,:)),'LineWidth',2) % left, y
plot(tspan,squeeze(vFeet(1,2,:)),'LineWidth',2) % right, x
plot(tspan,squeeze(vFeet(2,2,:)),'LineWidth',2) % right, y

xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend({'L_x','L_y','R_x','R_y'});
title('Feet Velocity')

% Plot leg joint angles over times
figure(3)
plot(tspan, z_out(1:4,:)*180/pi);
legend('q1','q2', 'q3', 'q4');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Angles')

% Plot desired feet position
figure(4)
subplot(2,1,1)
plot(tspan, footTraj([1,3],:));
xlabel('Time (s)'); ylabel('Position (m)'); legend({'L_x','R_x'});
title('Desired Feet Position')
subplot(2,1,2);
plot(tspan, footTraj([2,4],:));
xlabel('Time (s)'); ylabel('Position (m)'); legend({'L_y','R_y'});
title('Desired Feet Position')

% Plot desired feet velocity
figure(5)
subplot(2,1,1)
plot(tspan, footTraj([5,7],:));
xlabel('Time (s)'); ylabel('Velocty (m/s)'); legend({'L_x','R_x'});
title('Desired Feet Velocity')
subplot(2,1,2);
plot(tspan, footTraj([6,8],:));
xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend({'L_y','R_y'});
title('Desired Feet Velocity')

figure(7)
plot(tspan, rHip')

%% Animate Solution
% animateSol(sim, p, z_out)
figure(6); clf;

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
plot([-0.25 1],[ground_height ground_height],'k'); 
xlabel('x')
ylabel('y')

h_title = title('t = 0.0s');

axis equal
skip_frame = 1; % adjust animation frame rate

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