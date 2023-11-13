%% Script for testing aspects of the gait generator
% 11/06/2023

%% Create gait generator
setpath;
tStance = 0.5; % seconds
tSwing = 0.5; % seconds
gdPen = 0.25; % meters
avgVel = 0.1; % m/s
nomHip = [0; 0];
ctrlPts = [0.00 0.10 0.50 0.90 1.00;
           0.00 1.00 0.50 1.00 0.00];

obj = GaitGenerator(ctrlPts, nomHip, tStance, tSwing, gdPen, avgVel);

%%%%%%%%%%%%%%
% tests
%% singleBezier
t = 0:0.01:1;
bezOut = zeros(2, length(t));
for i = 1:length(t)
    bezOut(:, i) = obj.singleBezier(t(i));
end

figure(1); clf
plot(bezOut(1, :), bezOut(2, :))
hold on
scatter(obj.ctrlPts(1, :), obj.ctrlPts(2, :))
legend('Traj', 'CtrlPts')

%% hipGenerator
t = 0:0.01:10;
hipOut = zeros(2, length(t));
for i = 1:length(t)
    hipOut(:, i) = obj.hipGenerator(t(i));
end

figure(2); clf
plot(hipOut(1, :), hipOut(2, :))

%% groundContactGenerator
t = 0:0.01:4;
gdOut = zeros(4, length(t));
for i = 1:length(t)
    thisT = [mod(t(i) - (obj.tSwing + obj.tStance)*obj.phase, obj.tSwing + obj.tStance);
             mod(t(i), obj.tSwing + obj.tStance)];
    stridePortion = thisT/obj.tSwing; % when this is greater than 1, you are in stance
    stancePortion = (thisT - obj.tSwing)/obj.tStance;
    gdOut(:, i) = obj.groundContactGenerator(stancePortion);
end

figure(3); clf
subplot(2, 1, 1)
plot(t, gdOut(2, :), 'r')
hold on
plot(t, gdOut(4, :), 'b')
axis equal
legend('Left', 'Right')
subplot(2, 1, 2)
plot(t, gdOut(1, :), 'r')
hold on
plot(t, gdOut(3, :), 'b')
axis equal
legend('Left', 'Right')

%% footPos
t = 0:0.001:4;
footTrajOut = zeros(4, length(t));
inContact = zeros(2, length(t));
for i = 1:length(t)
    [footTrajOut(:, i), inContact(:, i)] = obj.footPatternGenerator(t(i));
end

figure(4); clf
subplot(2, 1, 1)
plot(footTrajOut(1, :), footTrajOut(2, :), 'r')
axis equal
subplot(2, 1, 2)
plot(footTrajOut(3, :), footTrajOut(4, :), 'b')
axis equal

figure(5); clf
subplot(2, 1, 1)
plot(t, footTrajOut(2, :), 'r')
hold on
plot(t, footTrajOut(4, :), 'b')
axis equal
legend('Left', 'Right')
ylabel('y (m)')
subplot(2, 1, 2)
plot(t, footTrajOut(1, :), 'r')
hold on
plot(t, footTrajOut(3, :), 'b')
axis equal
legend('Left', 'Right')
ylabel('x (m)')
xlabel('Time (s)')

%% globalFootPos
t = 0:0.01:1;
footTrajOut = zeros(4, length(t));
for i = 1:length(t)
    footTrajOut(:, i) = obj.globalFootPos(t(i));
end

% obj.plotFeetTraj(footTrajOut)

figure(7); clf
subplot(2, 1, 1)
plot(t, footTrajOut(2, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(4, :), 'b', 'LineWidth', 2)
plot(t, 0*t, 'k')
legend('Left', 'Right')
ylabel('y (m)')
subplot(2, 1, 2)
plot(t, footTrajOut(1, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(3, :), 'b', 'LineWidth', 2)
legend('Left', 'Right')
ylabel('x (m)')
xlabel('Time (s)')

%%
% animate this
figure(8); clf
subplot(2, 1, 1); hold on
plot(t, footTrajOut(2, :), 'r')
plot(t, footTrajOut(4, :), 'b')
% legend('Left', 'Right')
ylabel('y (m)')
subplot(2, 1, 2); hold on
plot(t, footTrajOut(1, :), 'r')
plot(t, footTrajOut(3, :), 'b')
% legend('Left', 'Right')
ylabel('x (m)')
xlabel('Time (s)')
for i = 1:length(t)
    subplot(2, 1, 1)
    top = plot(t(i), footTrajOut(2, i), t(i), footTrajOut(4, i), '*k');
    subplot(2, 1, 2)
    bottom = plot(t(i), footTrajOut(1, i), t(i), footTrajOut(3, i), '*k');

    pause(0.001)
    delete(top); delete(bottom)
end

%%
figure(9); clf
subplot(2, 1, 1); 
hold on
plot(footTrajOut(1, :), footTrajOut(2, :), 'r')
ylabel('y (m)')
subplot(2, 1, 2);
hold on
plot(footTrajOut(3, :), footTrajOut(4, :), 'b')
xlabel('x (m)')
ylabel('y (m)')
sgtitle('Foot Trajectories over Time')

for i = 1:length(t)
    subplot(2, 1, 1)
    top = plot(footTrajOut(1, i), footTrajOut(2, i), '*k');
    subplot(2, 1, 2)
    bot = plot(footTrajOut(3, i), footTrajOut(4, i), '*k');

    pause(0.01)
    delete(top); delete(bot)
end
