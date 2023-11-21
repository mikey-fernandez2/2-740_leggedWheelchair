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
            0.00 0.10 0.05 0.10 0.00]; % YOUR BEZIER PTS HERE
        
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
gains.D = 50;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.3;

t_swing= 1;
t_stance = 1;
phase_offset = 0.5;
ground_penetration = 0.1;
nomHip = [0 0.05];
avgVel = 0.05;

initAngles = [angle1_init, angle2_init, angle3_init, angle4_init];
times = [pre_buffer_time traj_time post_buffer_time];
gaitParams = [t_swing t_stance phase_offset ground_penetration nomHip avgVel];

%% Run Experiment
[output_data] = RunTrajectoryExperiment_er(initAngles, times, pts_foot,...                                      
                                        gains, duty_max, gaitParams);
% RunTrajectoryExperiment_er(initAngles, times, pts_foot, gains, duty_max, gaitParams)

% remove initial stuff
output_data = output_data(2000:4000, :);

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
dydex1 = output_data(:, 29);
ddxdes1 = output_data(:, 30);
ddydes1 = output_data(:, 31);

x2 = -output_data(:,32);         % actual foot position (negative due to direction motors are mounted)
y2 = output_data(:,33);         % actual foot position
dx2 = -output_data(:, 34);
dy2 = output_data(:, 35);
xdes2 = output_data(:,36);      % desired foot position (negative due to direction motors are mounted)
ydes2 = output_data(:,37);      % desired foot position 
dxdes2 = output_data(:, 38);
dydex2 = output_data(:, 39);
ddxdes2 = output_data(:, 40);
ddydes2 = output_data(:, 41);

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
