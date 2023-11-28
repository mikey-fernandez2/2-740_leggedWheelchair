classdef GaitGenerator
    properties
        legs = 2;
        phase = 0.5; % 50% offset in phase
        tSwing = 0.5; % seconds, the default swing time
        ctrlPts = zeros(2, 4); % these should be in [0, 1] in x to scale with stride length
        numPts = 4;
        tStance
        lenStride
        gdPen
        avgVel; % this should be a scalar
        nomHip; % this is the nominal position of the hip within the stride (0.5, 0.15), approx
    end
    % the ctrlPts should be [0, 0], *intermediate points*, [1, 0] to start
    % and end in contact with the ground

    methods
        function obj = GaitGenerator(ctrlPts, nomHip, tStance, tSwing, gdPen, avgVel)
            if exist('ctrlPts', 'var')
                assert(size(ctrlPts, 1) == 2, 'Bezier Control Points must be 2xn double matrix')
            end
            if nargin > 0
                obj.ctrlPts = ctrlPts;
                obj.numPts = length(ctrlPts);
                obj.nomHip = [nomHip; nomHip; zeros(8, 1)];
                obj.tStance = tStance;
                obj.tSwing = tSwing;
                obj.gdPen = gdPen;
                obj.avgVel = avgVel;

                % generate the stride length such that the foot doesn't move during stance
                % the avgVel should be equal to the distance the foot moves
                % backwards, relative to the hip, during stance
                % CoM distance forward is avgVel*tStance, so the stride
                % length should be this value
                obj.lenStride = avgVel*tStance;
            end
        end

        function bezPos = singleBezier(obj, t, n, ctrlPts)
            bezPos = [0; 0];
            for i = 0:(n - 1)
                bezPos = bezPos + nchoosek(n - 1, i)*(1 - t).^(n - i - 1).*t.^i.*ctrlPts(:, i + 1);
            end
        end

        function bezOut = BezierGenerator(obj, t)
            % both legs run the same trajectory, but there is a phase
            % difference - use different time indices from t = [t(1), t(2)]
            bezBothPos = [obj.singleBezier(t(1), obj.numPts, obj.ctrlPts); obj.singleBezier(t(2), obj.numPts, obj.ctrlPts)];
            bezBothVel = (obj.numPts - 1)*[obj.singleBezier(t(1), obj.numPts - 1, diff(obj.ctrlPts, 1, 2)); obj.singleBezier(t(2), obj.numPts - 1, diff(obj.ctrlPts, 1, 2))];
            bezBothAcc = (obj.numPts - 1)*(obj.numPts - 2)*[obj.singleBezier(t(1), obj.numPts - 2, diff(diff(obj.ctrlPts, 1, 2), 1, 2)); obj.singleBezier(t(2), obj.numPts - 2, diff(diff(obj.ctrlPts, 1, 2), 1, 2))];
            bezOut = [bezBothPos; bezBothVel; bezBothAcc];
        end

        function hipOut = hipGenerator(obj, t)
%             hipPos = obj.nomHip + [obj.avgVel*t; 0]; % STS: this line didn't work dimensionally
            hipPos = obj.nomHip(1:2) + [obj.avgVel*t; 0];
            hipVel = [obj.avgVel; 0];
            hipAcc = [0; 0];
            hipOut = [repmat(hipPos, 2, 1); repmat(hipVel, 2, 1); repmat(hipAcc, 2, 1)];
        end

        function gdOut = groundContactGenerator(obj, t)
            % this function uses a sine curve over the ground contact for
            % the y nominal, and the x nominal is the normalized t
            gdPosFcn = @(t) [1 - t;     -obj.gdPen*sin(t*pi)];
            gdVelFcn = @(t) [   -1;  -pi*obj.gdPen*cos(t*pi)];
            gdAccFcn = @(t) [    0; pi^2*obj.gdPen*sin(t*pi)];
            gdPos = [gdPosFcn(t(1))*(t(1) >= 0); gdPosFcn(t(2))*(t(2) >= 0)];
            gdVel = [gdVelFcn(t(1))*(t(1) >= 0); gdVelFcn(t(2))*(t(2) >= 0)];
            gdAcc = [gdAccFcn(t(1))*(t(1) >= 0); gdAccFcn(t(2))*(t(2) >= 0)];
            gdOut = [gdPos; gdVel; gdAcc];
        end

        function [footKinematics_hip, inContact] = footPatternGenerator(obj, tRaw)
            % need to normalize this time value, for the two legs
            % this includes the phase offset between the legs
            t = [mod(tRaw - (obj.tSwing + obj.tStance)*obj.phase, obj.tSwing + obj.tStance);
                mod(tRaw, obj.tSwing + obj.tStance)];

            inSwing = repmat(repelem(t < obj.tSwing, 2), [3, 1]); % this is a logical vector that determines whether each leg should be in contact or not
            swingPortion = t/obj.tSwing; % when this is greater than 1, you are in stance
            stancePortion = (t - obj.tSwing)/obj.tStance; % this is scaling of stance between 0 and 1

            % fprintf('t: %0.3f\n', tRaw);
            % fprintf('\tinSwing: %d | stridePortion: %f | stancePortion % f\n', [inSwing([1, 3]) stridePortion stancePortion]');

            footKinematics_hip_relative = (obj.BezierGenerator(swingPortion).*inSwing + obj.groundContactGenerator(stancePortion).*(~inSwing));
            footKinematics_hip = (footKinematics_hip_relative - obj.nomHip).*repmat([obj.lenStride; 1], 6, 1);
            % foot position, relative to the hip, is the linear combination of stance and stride position
            inContact = ~inSwing([1, 3]);
        end

        function [footTraj, inContact] = globalFootPos(obj, tRaw)
            [footKinematics_hip, inContact] = obj.footPatternGenerator(tRaw);
            hipKinematics = obj.hipGenerator(tRaw); % get the hip kinematics

            footTraj = hipKinematics + footKinematics_hip;
        end

        function plotFeetTraj(obj, footTrajs)
            figure(100); clf;
            plot(footTrajs(1, :), footTrajs(2, :))
            hold on
            plot(footTrajs(3, :), footTrajs(4, :))
            xlabel('x (m)')
            ylabel('y (m)')
            axis equal
            legend('Left', 'Right')
        end
    end
end