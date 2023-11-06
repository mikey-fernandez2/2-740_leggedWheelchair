classdef GaitGenerator
    properties
        legs = 2;
        tStride = 0.5; % seconds, the default stride time
        ctrlPts = zeros(2, 4); % these should be in [0, 1] in x to scale with stride length
        numPts = 4;
        tStance
        lenStride
        gdPen
        avgVel
    end

    methods
        function obj = GaitGenerator(ctrlPts, tStance, lenStride, gdPen, avgVel)
            if exist('ctrlPts', 'var')
                assert(size(ctrlPts, 1) == 2, 'Bezier Control Points must be 2xn double matrix')
            end
            if nargin > 0
                obj.ctrlPts = ctrlPts;
                obj.numPts = length(ctrlPts);
                obj.tStance = tStance;
                obj.lenStride = lenStride;
                obj.gdPen = gdPen;
                obj.avgVel = avgVel;
            end
        end

        function bezHandle = BezierGenerator(obj)
            syms t real
            bezOut = [0; 0];
            for i = 1:obj.numPts

    end
end