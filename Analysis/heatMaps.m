% Generate pairwise heatmaps between successful runs
close all;

% load('Results/results.mat');

varNames_all = resultsTable.Properties.VariableNames;
varNamesUse = varNames_all(4:8);


for xVar = 1:4
    for yVar = xVar + 1:5
        generateHeatMap(resultsTable, char(varNamesUse(xVar)), char(varNamesUse(yVar)));
    end
end

for xVar = 1:3
    for yVar = xVar + 1:4
        for zVar = yVar + 1:5
            generateScatter(resultsTable, gaitParams.([char(varNamesUse(xVar)) 'Range']), gaitParams.([char(varNamesUse(yVar)) 'Range']), ...
                gaitParams.([char(varNamesUse(zVar)) 'Range']), char(varNamesUse(xVar)), char(varNamesUse(yVar)), char(varNamesUse(zVar)));
        end
    end
end

function [outputX, outputPitch] = reorderSmoothness(tbl, xRange, yRange, zRange, xVar, yVar, zVar)
    outputX = zeros(length(yRange), length(xRange), length(zRange)); outputPitch = zeros(length(yRange), length(xRange), length(zRange));
    for x = 1:length(xRange)
        for y = 1:length(yRange)
            for z = 1:length(zRange)
                xYes = tbl.(xVar) == xRange(x); yYes = tbl.(yVar) == yRange(y); zYes = tbl.(zVar) == zRange(z);
                idx = find(xYes & yYes & zYes);
                xSmooth = nanmean(tbl.xSmooth(idx)); pitchSmooth = nanmean(tbl.pitchSmooth(idx));
                outputX(y, x, z) = xSmooth; outputPitch(y, x, z) = pitchSmooth;
                % yes the indices are weird but they're what's required to match meshgrid() with 3 inputs
            end
        end
    end
end

function generateScatter(resultsTable, xRange, yRange, zRange, xVar, yVar, zVar)
    [outputX, outputPitch] = reorderSmoothness(resultsTable, xRange, yRange, zRange, xVar, yVar, zVar);
    [X, Y, Z] = meshgrid(xRange, yRange, zRange);
    figure;
    scatter3(X(:), Y(:), Z(:), 1000, -outputX(:), 'filled'); colorbar;
    xlabel(xVar)
    ylabel(yVar)
    zlabel(zVar)
    title(['Impact of ' xVar ', ' yVar ' and ' zVar ' on x-smoothness'])
    figure;
    scatter3(X(:), Y(:), Z(:), 1000, -outputPitch(:), 'filled'); colorbar;
    xlabel(xVar)
    ylabel(yVar)
    zlabel(zVar)
    title(['Impact of ' xVar ', ' yVar ' and ' zVar ' on \phi-smoothness'])
end

function generateHeatMap(tbl, xVar, yVar)
    figure;
    subplot(2, 1, 1)
    heatmap(tbl, xVar, yVar, 'ColorVariable', 'xSmooth');
    subplot(2, 1, 2)
    heatmap(tbl, xVar, yVar, 'ColorVariable', 'pitchSmooth');
    sgtitle(['Impact of ' xVar ' and ' yVar ' on smoothness'])
end