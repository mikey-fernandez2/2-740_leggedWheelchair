% This is the main MATLAB script for Lab 5.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
%const_point = [0; 0]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
%pts_foot = repmat(const_point,1,5);
close; clear;clc;  
pts_foot = [0.00 0.10 0.50 0.90 1.00;
            0.00 0.05 0.025 0.05 0.00]; % YOUR BEZIER PTS HERE
        
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = pi/2; 
angle3_init = 0;
angle4_init = -pi/2; 

% Total experiment time is buffer,trajectory,buffer
traj_time         = 10;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 2;

% Gains for impedance controller
gains.K = 100;
gains.D = 2;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max = 1;
t_swing= 1.0;
t_stance = 1.0;
phase_offset = 0.5;
ground_penetration = 0.01;
avgVel = 0.15;
nomHip = [-.1/(t_stance*avgVel) + 1, 0.125];

gaitAsymmetry = 0; % set negative to turn left, positive to turn right - STRICTLY in [-1, 1]

initAngles = [angle1_init, angle2_init, angle3_init, angle4_init];
times = [pre_buffer_time traj_time post_buffer_time];
gaitParams = [t_swing t_stance phase_offset ground_penetration nomHip avgVel gaitAsymmetry];

%% Run Experiment
[output_data] = RunTrajectoryExperiment_er(initAngles, times, pts_foot,...                                      
                                        gains, duty_max, gaitParams);
% RunTrajectoryExperiment_er(initAngles, times, pts_foot, gains, duty_max, gaitParams)


%% Extract data
t = output_data(:,1);          % time
pos1 = output_data(:,2);       % position
vel1 = output_data(:,3);       % velocity
cur1 = output_data(:,4);       % current
dcur1 = output_data(:,5);      % desired current
duty1 = output_data(:,6);      % command

pos2 = output_data(:,7);       % position
vel2 = output_data(:,8);       % velocity
cur2 = output_data(:,9);       % current
dcur2 = output_data(:,10);     % desired current
duty2 = output_data(:,11);     % command

pos3 = output_data(:,12);       % position
vel3 = output_data(:,13);       % velocity
cur3 = output_data(:,14);       % current
dcur3 = output_data(:,15);     % desired current
duty3 = output_data(:,16);     % command

pos34 = output_data(:,17);       % position
vel4 = output_data(:,18);       % velocity
cur4 = output_data(:,19);       % current
dcur4 = output_data(:,20);     % desired current
duty4 = output_data(:,21);     % command

x1 = -output_data(:,22);         % actual foot position (negative due to direction motors are mounted)
y1 = output_data(:,23);         % actual foot position
dx1 = -output_data(:, 24);
dy1 = output_data(:, 25);
xdes1 = output_data(:,26);      % desired foot position (negative due to direction motors are mounted)
ydes1 = output_data(:,27);      % desired foot position
dxdes1 = output_data(:, 28);
dydes1 = output_data(:, 29);
ddxdes1 = output_data(:, 30);
ddydes1 = output_data(:, 31);

x2 = -output_data(:,32);         % actual foot position (negative due to direction motors are mounted)
y2 = output_data(:,33);         % actual foot position
dx2 = -output_data(:, 34);
dy2 = output_data(:, 35);
xdes2 = output_data(:,36);      % desired foot position (negative due to direction motors are mounted)
ydes2 = output_data(:,37);      % desired foot position 
dxdes2 = output_data(:, 38);
dydes2 = output_data(:, 39);
ddxdes2 = output_data(:, 40);
ddydes2 = output_data(:, 41);

acc_x = output_data(:, 42);
acc_y = output_data(:, 43);
acc_z = output_data(:, 44);
gyro_x = output_data(:, 45);
gyro_y = output_data(:, 46);
gyro_z = output_data(:, 47);

%% Plot foot vs desired
figure(3); clf;
subplot(211); hold on
plot(t,xdes1,'r-'); plot(t,x1,'k');
plot(t,xdes2,'b-'); plot(t,x2,'g');
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired X1','Actual X1','Desired X2','Actual X2'});

subplot(212); hold on
plot(t,ydes1,'r-'); plot(t,y1,'k');
plot(t,ydes2,'b-'); plot(t,y2,'g');
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired Y1','Actual Y1','Desired Y2','Actual Y2'});

figure(4); clf; hold on
plot(xdes1,ydes1,'r-'); plot(x1,y1,'k');
plot(xdes2,ydes2,'b-'); plot(x2,y2,'g');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired 1','Actual 1','Desired 2','Actual 2'});

figure(5); clf; hold on
plot(dxdes1,dydes1,'r'); 
% plot(dx1,dy1,'k');
plot(dxdes2,dydes2,'b'); 
% plot(dx2,dy2,'g');
xlabel('dX (m/s)'); ylabel('dY (ms)'); legend({'Desired 1','Actual 1','Desired 2','Actual 2'});

figure(6); clf;
subplot(2, 1, 1); hold on
plot(t, dxdes1,'r-'); plot(t, dxdes2,'b-'); plot(t, dx1,'k--'); plot(t, dx2,'g--');
ylabel('dx (m/s)')
subplot(2, 1, 2); hold on
plot(t, dydes1,'r-'); plot(t, dydes2,'b-');
xlabel('t (s)'); ylabel('dy (m/s)'); legend({'Desired 1','Actual 1','Desired 2','Actual 2'});

saving_models = true; 
if saving_models == true %saving models for use in other scripts
    %clearvars -except thetas stiffness_model stiffness_tr stress_model stress_tr error_stress error_stiffness stress_outputs stiff_outputs stress_predictions stiff_predictions
    str = sprintf('outputData_%2.0f_%2.0f_%2.1f_%2.1f_%1.2f_%1.3f.mat',gains.K,gains.D,t_swing,t_stance,avgVel,ground_penetration);
    save(str);
end

