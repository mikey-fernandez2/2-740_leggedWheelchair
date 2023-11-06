classdef GaitGenerator
    properties
        legs = 2;
        phase = 0.5; % 50% offset in phase
        tStride = 0.5; % seconds, the default stride time
        ctrlPts = zeros(2, 4); % these should be in [0, 1] in x to scale with stride length
        numPts = 4;
        tStance
        lenStride
        gdPen
        avgVel; % this should be a 2x1 vector
        nomHip; % this is the nominal position of the hip
    end
    % the ctrlPts should be [0, 0], *intermediate points*, [1, 0] to start
    % and end in contact with the ground

    methods
        function obj = GaitGenerator(ctrlPts, nomHip, tStance, gdPen, avgVel)
            if exist('ctrlPts', 'var')
                assert(size(ctrlPts, 1) == 2, 'Bezier Control Points must be 2xn double matrix')
            end
            if nargin > 0
                obj.ctrlPts = ctrlPts;
                obj.numPts = length(ctrlPts);
                obj.nomHip = nomHip;
                obj.tStance = tStance;
                obj.gdPen = gdPen;
                obj.avgVel = avgVel;
                obj.lenStride = avgVel(1)*(tStance + obj.tStride)/tStance;
            end
        end

        function bezOut = singleBezier(obj, t)
            bezOut = [0; 0];
            n = obj.numPts;
            for i = 0:(n - 1)
                bezOut = bezOut + nchoosek(n - 1, i)*(1 - t).^(n - i - 1).*t.^i.*obj.ctrlPts(:, i + 1);
            end
        end

        function bezBoth = BezierGenerator(obj, t)
            % both legs run the same trajectory, but there is a phase
            % difference - use different time indices
            bezBoth = [obj.singleBezier(t(1)); obj.singleBezier(t(2))];
        end

        function hipTraj = hipGenerator(obj, t)
            hipTraj = obj.nomHip + [obj.avgVel(1)*t; obj.avgVel(2)*t];
        end

        function gdBoth = groundContactGenerator(obj, t)
            % this function uses a sine curve over the ground contact for
            % the y nominal, and the x nominal is the normalized t
            gdOne = @(t) [1 - t; -obj.gdPen*sin(t*pi)];
            gdBoth = [gdOne(t(1) - 1)*(t(1) > 1); gdOne(t(2) - 1)*(t(2) > 1)];
        end

        function footTraj_hip = footPatternGenerator(obj, tRaw)
            % need to normalize this time value, for the two legs
            % this includes the phase offset between the legs
            t = [mod(tRaw - (obj.tStride + obj.tStance)*obj.phase, obj.tStride + obj.tStance);
                mod(tRaw, obj.tStride + obj.tStance)];

            inStride = repelem(t <= obj.tStride/(obj.tStride + obj.tStance), 2); % this is a logical vector that determines whether each leg should be in contact or not
            stridePortion = t/obj.tStride; % when this is greater than 1, you are in stance

            footTraj_hip = obj.BezierGenerator(stridePortion).*inStride + obj.groundContactGenerator(stridePortion).*(~inStride);
            % foot position, relative to the hip, is the linear combination of stance and stride position
        end

        function footTraj = globalFootPos(obj, tRaw)
            footTraj_hip = obj.footPatternGenerator(tRaw);
            hipPos = repmat(obj.hipGenerator(tRaw), 2, 1); % get the hip position

            % during ground contact the foot shouldn't be moving; adjust
            % for this at the hipPos
            groundContact = repelem([footTraj_hip(2); footTraj_hip(4)], 2) < 0;

            footTraj = hipPos + footTraj_hip.*repmat([obj.lenStride; 1], 2, 1);
        end

        function plotFeetTraj(obj, footTrajs)
            figure(1); clf;
            plot(footTrajs(1, :), footTrajs(2, :))
            hold on
            plot(footTrajs(3, :), footTrajs(4, :))
            xlabel('x (m)')
            ylabel('y (m)')
            legend('Left', 'Right')
        end
    end
end