%% Script for testing aspects of the gait generator
% 11/06/2023

%% Create gait generator
setpath;
tStance = 0.75; % seconds
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
    bezOut(:, i) = obj.singleBezier(t(i), obj.numPts, obj.ctrlPts);
end

figure(1); clf
plot(bezOut(1, :), bezOut(2, :))
hold on
scatter(obj.ctrlPts(1, :), obj.ctrlPts(2, :))
legend('Traj', 'CtrlPts')

%% derivatives of Bezier
t = 0:0.001:1;
bezOut = zeros(12, length(t));
for i = 1:length(t)
    bezOut(:, i) = obj.BezierGenerator([t(i), t(i) + (obj.tSwing + obj.tStance)*obj.phase]);
end

figure(100); clf
subplot(3, 1, 1)
plot(bezOut(1, :), bezOut(2, :))
hold on
scatter(obj.ctrlPts(1, :), obj.ctrlPts(2, :))
plot(bezOut(1, 1), bezOut(2, 1), '*')
ylabel('pos')
subplot(3, 1, 2)
plot(bezOut(5, :), bezOut(6, :))
hold on
plot(bezOut(5, 1), bezOut(6, 1), '*')
scatter((obj.numPts - 1)*diff(obj.ctrlPts(1, :), 1, 2), (obj.numPts - 1)*diff(obj.ctrlPts(2, :), 1, 2))
ylabel('vel')
subplot(3, 1, 3)
plot(bezOut(9, :), bezOut(10, :))
hold on
plot(bezOut(9, 1), bezOut(10, 1), '*')
scatter((obj.numPts - 1)*(obj.numPts - 2)*diff(diff(obj.ctrlPts(1, :), 1, 2), 1, 2), (obj.numPts - 1)*(obj.numPts - 2)*diff(diff(obj.ctrlPts(2, :), 1, 2), 1, 2))
ylabel('acc')
legend('Traj', 'CtrlPts')

%% hipGenerator
t = 0:0.01:10;
hipOut = zeros(12, length(t));
for i = 1:length(t)
    hipOut(:, i) = obj.hipGenerator(t(i));
end

figure(2); clf
subplot(3, 1, 1)
plot(hipOut(1, :), hipOut(2, :))
ylabel('hip pos')
subplot(3, 1, 2)
plot(hipOut(5, :), hipOut(6, :))
ylabel('hip vel')
subplot(3, 1, 3)
plot(hipOut(9, :), hipOut(10, :))
ylabel('hip accel')

%% groundContactGenerator
t = 0:0.01:4;
gdOut = zeros(12, length(t));
for i = 1:length(t)
    thisT = [mod(t(i) - (obj.tSwing + obj.tStance)*obj.phase, obj.tSwing + obj.tStance);
             mod(t(i), obj.tSwing + obj.tStance)];
    stridePortion = thisT/obj.tSwing; % when this is greater than 1, you are in stance
    stancePortion = (thisT - obj.tSwing)/obj.tStance;
    gdOut(:, i) = obj.groundContactGenerator(stancePortion);
end

figure(3); clf
subplot(2, 3, 1)
plot(t, gdOut(2, :), 'r')
hold on
plot(t, gdOut(4, :), 'b')
axis equal
ylabel('y pos')
legend('Left', 'Right')
subplot(2, 3, 4)
plot(t, gdOut(1, :), 'r')
hold on
plot(t, gdOut(3, :), 'b')
axis equal
ylabel('x pos')
legend('Left', 'Right')
subplot(2, 3, 2)
plot(t, gdOut(6, :), 'r')
hold on
plot(t, gdOut(8, :), 'b')
axis equal
ylabel('y vel')
legend('Left', 'Right')
subplot(2, 3, 5)
plot(t, gdOut(5, :), 'r')
hold on
plot(t, gdOut(7, :), 'b')
axis equal
ylabel('x vel')
legend('Left', 'Right')
subplot(2, 3, 3)
plot(t, gdOut(10, :), 'r')
hold on
plot(t, gdOut(12, :), 'b')
axis equal
ylabel('y acc')
legend('Left', 'Right')
subplot(2, 3, 6)
plot(t, gdOut(9, :), 'r')
hold on
plot(t, gdOut(11, :), 'b')
axis equal
ylabel('x acc')
legend('Left', 'Right')

%% footPos
t = 0:0.001:4;
footTrajOut = zeros(12, length(t));
inContact = zeros(2, length(t));
for i = 1:length(t)
    [footTrajOut(:, i), inContact(:, i)] = obj.footPatternGenerator(t(i));
end

figure(4); clf
subplot(2, 3, 1)
plot(t, footTrajOut(2, :), 'r')
hold on
plot(t, footTrajOut(4, :), 'b')
axis equal
ylabel('y pos')
legend('Left', 'Right')
subplot(2, 3, 4)
plot(t, footTrajOut(1, :), 'r')
hold on
plot(t, footTrajOut(3, :), 'b')
axis equal
ylabel('x pos')
legend('Left', 'Right')
subplot(2, 3, 2)
plot(t, footTrajOut(6, :), 'r')
hold on
plot(t, footTrajOut(8, :), 'b')
ylabel('y vel')
legend('Left', 'Right')
subplot(2, 3, 5)
plot(t, footTrajOut(5, :), 'r')
hold on
plot(t, footTrajOut(7, :), 'b')
ylabel('x vel')
legend('Left', 'Right')
subplot(2, 3, 3)
plot(t, footTrajOut(10, :), 'r')
hold on
plot(t, footTrajOut(12, :), 'b')
ylabel('y acc')
legend('Left', 'Right')
subplot(2, 3, 6)
plot(t, footTrajOut(9, :), 'r')
hold on
plot(t, footTrajOut(11, :), 'b')
ylabel('x acc')
legend('Left', 'Right')

% figure(4); clf
% subplot(2, 1, 1)
% plot(footTrajOut(1, :), footTrajOut(2, :), 'r')
% axis equal
% subplot(2, 1, 2)
% plot(footTrajOut(3, :), footTrajOut(4, :), 'b')
% axis equal
% 
% figure(5); clf
% subplot(2, 1, 1)
% plot(t, footTrajOut(2, :), 'r')
% hold on
% plot(t, footTrajOut(4, :), 'b')
% axis equal
% legend('Left', 'Right')
% ylabel('y (m)')
% subplot(2, 1, 2)
% plot(t, footTrajOut(1, :), 'r')
% hold on
% plot(t, footTrajOut(3, :), 'b')
% axis equal
% legend('Left', 'Right')
% ylabel('x (m)')
% xlabel('Time (s)')

%% globalFootPos
t = 0:0.001:4;
footTrajOut = zeros(12, length(t));
for i = 1:length(t)
    [footTrajOut(:, i), ~] = obj.globalFootPos(t(i));
end

% obj.plotFeetTraj(footTrajOut)

figure(7); clf
subplot(2, 3, 1)
plot(t, footTrajOut(2, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(4, :), 'b', 'LineWidth', 2)
plot(t, 0*t, 'k')
legend('Left', 'Right')
ylabel('y (m)')
subplot(2, 3, 4)
plot(t, footTrajOut(1, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(3, :), 'b', 'LineWidth', 2)
legend('Left', 'Right')
ylabel('x (m)')
xlabel('Time (s)')
subplot(2, 3, 2)
plot(t, footTrajOut(6, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(8, :), 'b', 'LineWidth', 2)
legend('Left', 'Right')
ylabel('dy (m/s)')
subplot(2, 3, 5)
plot(t, footTrajOut(5, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(7, :), 'b', 'LineWidth', 2)
legend('Left', 'Right')
ylabel('dx (m/s)')
xlabel('Time (s)')
subplot(2, 3, 3)
plot(t, footTrajOut(10, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(12, :), 'b', 'LineWidth', 2)
legend('Left', 'Right')
ylabel('ddy (m/s^2)')
subplot(2, 3, 6)
plot(t, footTrajOut(9, :), 'r', 'LineWidth', 2)
hold on
plot(t, footTrajOut(11, :), 'b', 'LineWidth', 2)
legend('Left', 'Right')
ylabel('ddx (m/s^2)')
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

%% Functions