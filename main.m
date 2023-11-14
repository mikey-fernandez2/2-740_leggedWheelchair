%% 2.740 Team 1 Legged Wheelchair Main Script

clc; clear; close all;

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
ma = 0.200; % 100 grams per wheel (6 oz. each) - weigh them!
mb = m5 + 2.0; % total guess; mass of motors plus mounting hardware
b = 0.267; % 12" 80/20

r = 0.035; % wheels from 2.007 lab

l_OA = 0.011; % same as l_OA from lab (not quite accurate)
l_OB = 0.042; % same as l_OB from lab (not quite accurate)
l_AC = 0.096; % same as l_AC from lab (not quite accurate)
l_DE = 0.091; % same as l_DE from lab (not quite accurate)
l_O_m1 = 0.032; % l_O_m1
l_B_m2 = 0.0344; % l_B_m2
l_A_m3 = 0.0622; % l_A_m3 from lab
l_C_m4 = 0.0610; % l_C_m4 from lab (revisit this)
l_cb = 0.1524; % 6" (doesn't account for amount hanging off past connection points)

I1 = 25.1 * 10^-6;
I2 = 53.5 * 10^-6;
I3 = 9.25 * 10^-6;
I4 = 22.176 * 10^-6;
I_A = 0.5 * ma * r^2; % thin solid disk
I_B = 1 * 10^-3; % truly a random guess. Need to do more calcs

N = 18.75;
Ir = 0.0035/N^2;
g = 9.81;

% Ground contact properties
restitution_coeff = 0.1;
friction_coeff = 0.3;
ground_height = 0;

% Parameter vector
p = [m1 m2 m3 m4 ma mb I1 I2 I3 I4 I_A I_B l_OA l_OB l_AC l_DE b l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_cb r Ir N mU I_U lU phiU g]';

%% Simulation parameters
sim = struct();

sim.dt = 0.001;
sim.tf = 10;
sim.num_steps = floor(sim.tf/sim.dt);
sim.tspan = linspace(0, sim.tf, sim.num_steps);

% Ground contact properties
sim.restitution_coeff = restitution_coeff;
sim.friction_coeff = friction_coeff;
sim.ground_height = ground_height;

% order of generalized coordinates [th1  ; th2  ; th3  ; th4  ; th5  ; x  ; y  ; phi  ];
sim.z0 = [pi/8; pi/4; pi/8; pi/4; pi/6; 0.5; 0.15; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%% Gait parameter ranges
tStanceRange = 0.1:0.1:1; % seconds
gdPenRange = 0.1:0.05:0.5; % meters (scaling?)
avgVelRange = 0.1:0.1:1; % m/s (scaling?)
stiffRange = 10:10:100; % N/m
dampRange = 1:10; % N-s/m

% fixed gait parameters
tSwing = 0.5; % seconds
nomHip = [0; 0]; % nominal hip position
ctrlPts = [0.00 0.10 0.50 0.90 1.00;
           0.00 1.00 0.50 1.00 0.00]; % control points for Bezier trajectory, normalized in [0, 1]

storedVals = {'xSmooth', 'pitchSmooth', 'tStance', 'gdPen', 'avgVel', 'K', 'D'};
resultsTable = cell2table(cell(0, length(storedVals)), 'VariableNames', storedVals);

ctrlStruct = struct();
for tStance = tStanceRange
    for gdPen = gdPenRange
        for avgVel = avgVelRange
            for K = stiffRange
                for D = dampRange
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
                    theseResults = {xSmooth, pitchSmooth, tStance, gdPen, avgVelVel, K, D};
                    resultsTable = [resultsTable; theseResults];
                end
            end
        end
    end
end
