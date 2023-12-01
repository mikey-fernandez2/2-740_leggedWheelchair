%% 2.740 Team 1 Legged Wheelchair Main Script

clc; clear; close all;

%%
setpath  % add subfolders

%% Define fixed paramters
% Lengths in meters, angles in radians, mass in kg
lU = 0.110; % distance from wheel center to "user" CoM
phiU = deg2rad(80); % angle from axis to "user" CoM
mU = 0.03; % 30 grams (2 oz.)
I_U = 1/12*mU*lU^2; % approximate as slender rod
m1 = .0393 + .2; % from lab
m2 = .0368; % from lab
m3 = .00783; % from lab
m4 = .0155; % from lab
m5 = 0.0424*12/2.205; % 12" length of 80/20 1010 profile
m_axle = 0.25*pi*(.25*.0254)^2*5*.0254*2710; % aluminum
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

I1 = 25.1*10^-6;
I2 = 53.5*10^-6;
I3 = 9.25*10^-6;
I4 = 22.176*10^-6;
I_A = ma*r^2; % thin solid disk
I_B = 1/12*m5*((.0254)^2+(12*.0254)^2)+.450*0.11^2; % inertia of a rectangle plus point inertia for motors

N = 18.75;
Ir = 0.0035/N^2;
g = 9.81;

% Ground contact properties
restitution_coeff = 0.1;
friction_coeff = 0.7;
ground_height = 0;

% Parameter vector
p = [m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';
pNames = '[m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';

%% Simulation parameters
sim = struct();

sim.dt = 0.001;
sim.tf = 5;
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
sim.z0 = [deg2rad(35); deg2rad(-90); deg2rad(-35); deg2rad(90); deg2rad(40); 0; 0.075; 0; 0; 0; 0; 0; 0; 0; 0; 0];

sim.p = p; % save the parameter vector

%% Gait parameter ranges
tStanceRange = 0.5:0.1:1; % seconds
gdPenRange = 0:0.02:0.1; % meters (scaling?)
avgVelRange = 0.05:0.05:0.3; % m/s (scaling?)
stiffRange = 0:100:500; % N/m
dampRange = 0:10:50; % N-s/m

numRuns = length(tStanceRange)*length(gdPenRange)*length(avgVelRange)*length(stiffRange)*length(dampRange);

% fixed gait parameters
tSwing = 0.5; % seconds
ctrlPts = [0.00 0.100 0.500 0.900 1.00;
           0.00 0.075 0.050 0.075 0.00]; % control points for Bezier trajectory, normalized in [0, 1]

% save the gait parameters
gaitParams = struct();
gaitParams.tStanceRange = tStanceRange;
gaitParams.gdPenRange = gdPenRange;
gaitParams.avgVelRange = avgVelRange;
gaitParams.KRange = stiffRange;
gaitParams.DRange = dampRange;
gaitParams.tSwing = tSwing;
gaitParams.ctrlPts = ctrlPts;

%% Simulate
storedVals = {'xSmooth', 'ySmooth', 'pitchSmooth', 'fracDesiredVelocity', 'tStance', 'gdPen', 'avgVel', 'K', 'D', 'Success'};
resultsTable = cell2table(cell(0, length(storedVals)), 'VariableNames', storedVals);

ctrlStruct = struct();
iters = 1; startTime = tic;
for tStance = tStanceRange
    for gdPen = gdPenRange
        for avgVel = avgVelRange
            for K = stiffRange
                for D = dampRange
                    if not(mod(iters, 10))
                        fprintf('Run %3d of %3d (%06.2f sec): %4.3f | %4.3f | %4.3f | %4.3f | %4.3f\n', [iters, numRuns, toc(startTime), tStance, gdPen, avgVel, K, D])
                    end

                    fullGaitTime = tStance + tSwing;
                    idxRecord = round(fullGaitTime/sim.dt);
                    desiredDistTraveled = (sim.tf - fullGaitTime)*avgVel;

                    nomHip = [-0.1/(tStance*avgVel) + 1; 0.125];
                    
                    % get the settings for this run
                    gaitGen = GaitGenerator(ctrlPts, nomHip, tStance, tSwing, gdPen, avgVel);
                    ctrlStruct.gait = gaitGen;
                    ctrlStruct.K = K;
                    ctrlStruct.D = D;

                    % simulate with these settings
                    [z_out, dz_out] = simulate(sim, p, ctrlStruct);

                    % calculate and store the smoothness values
                    % get the acceleration of the user
                    try
                        accelUser = acceleration_user(z_out(:, idxRecord:end), dz_out(:, idxRecord:end), p);
                        xAccel = accelUser(1, :); yAccel = accelUser(2, :); pitchAccel = accelUser(3, :);
                        % calculate smoothness
                        xSmooth = mean(abs(xAccel)); ySmooth = mean(abs(yAccel)); pitchSmooth = mean(abs(pitchAccel));
    
                        fractionDistTraveled = (z_out(6, end) - z_out(6, idxRecord))/desiredDistTraveled;
                    catch
                        z_out(5, end) = pi; % auto-fail if there are issues with the above
                        fractionDistTraveled = -1; % even if there is short cirtuciting in the or, add this 
                    end
    
                    % failure conditions
                    success = true;
                    if any(z_out(5, idxRecord:end) > pi/2) || any(z_out(5, idxRecord:end) < asin(-r/b)) || fractionDistTraveled < 0
                        success = false;
                    end

                    % store results in the table
                    % {'xSmooth', 'ySmooth', 'pitchSmooth', 'fracDesiredVelocity', 'tStance', 'gdPen', 'avgVel', 'K', 'D', 'Success'};
                    theseResults = {xSmooth, ySmooth, pitchSmooth, fractionDistTraveled, tStance, gdPen, avgVel, K, D, success};
                    resultsTable = [resultsTable; theseResults];

                    iters = iters + 1;
                end
            end
        end
    end
end

%
fprintf('Run %3d of %3d (%06.2f sec): %4.3f | %4.3f | %4.3f | %4.3f | %4.3f\n', [iters - 1, numRuns, toc(startTime), tStance, gdPen, avgVel, K, D])
save('Results/results.mat', 'resultsTable', 'sim', 'gaitParams', 'pNames')
