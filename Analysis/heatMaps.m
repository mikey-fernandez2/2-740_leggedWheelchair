% Generate pairwise heatmaps between successful runs
close all;

load('Results/results.mat');
successIdx = logical(resultsTable.Success);
resultsTable.xSmooth(~successIdx) = NaN; resultsTable.pitchSmooth(~successIdx) = NaN;
resultsTable.xSmooth(successIdx) = -resultsTable.xSmooth(successIdx); resultsTable.pitchSmooth(successIdx) = -resultsTable.pitchSmooth(successIdx);

varNames_all = resultsTable.Properties.VariableNames;
varNamesUse = varNames_all(5:9);


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
    video_xSmooth = VideoWriter(['Results/scatters/scatter_xSmooth_' xVar '_' yVar '_' zVar '.mp4'], 'MPEG-4');
    video_pitchSmooth = VideoWriter(['Results/scatters/scatter_pitchSmooth_' xVar '_' yVar '_' zVar '.mp4'], 'MPEG-4');
    [outputX, outputPitch] = reorderSmoothness(resultsTable, xRange, yRange, zRange, xVar, yVar, zVar);
    [X, Y, Z] = meshgrid(xRange, yRange, zRange);

    numFrames = 180;
    open(video_xSmooth)
    figure('Position', [100, 100, 800, 600]);
    scatter3(X(:), Y(:), Z(:), 1000, outputX(:), 'filled', 'MarkerFaceAlpha', 0.75); colorbar;
    xlabel(xVar, 'FontSize', 15)
    ylabel(yVar, 'FontSize', 15)
    zlabel(zVar, 'FontSize', 15)
    title(['Impact of ' xVar ', ' yVar ', and ' zVar ' on x-smoothness'], 'FontSize', 20)
    for frame = 1:numFrames
        camorbit(360/numFrames, 0, 'data');
        camzoom(1)
        writeVideo(video_xSmooth, getframe(gcf))
    end
    close(video_xSmooth)
    open(video_pitchSmooth)
    figure('Position', [100, 100, 800, 600]);
    scatter3(X(:), Y(:), Z(:), 1000, outputPitch(:), 'filled', 'MarkerFaceAlpha', 0.75); colorbar;
    xlabel(xVar, 'FontSize', 15)
    ylabel(yVar, 'FontSize', 15)
    zlabel(zVar, 'FontSize', 15)
    title(['Impact of ' xVar ', ' yVar ', and ' zVar ' on \phi-smoothness'], 'FontSize', 20)
    for frame = 1:numFrames
        camorbit(360/numFrames, 0, 'data');
        camzoom(1)
        writeVideo(video_pitchSmooth, getframe(gcf))
    end
    close(video_pitchSmooth)
end

function generateHeatMap(tbl, xVar, yVar)
    figure('Position', [100, 100, 1000, 750]);
    subplot(2, 1, 1)
    heatmap(tbl, xVar, yVar, 'ColorVariable', 'xSmooth', 'XLabel', '', 'Title', 'Mean Smoothness in x (m/s^2)', 'FontSize', 15, 'CellLabelColor', 'none');
    subplot(2, 1, 2)
    heatmap(tbl, xVar, yVar, 'ColorVariable', 'pitchSmooth', 'Title', 'Mean Smoothness in Pitch (rad/s^2)', 'FontSize', 15, 'CellLabelColor', 'none');
    sgtitle(['Impact of ' xVar ' and ' yVar ' on smoothness'], 'FontSize', 20)
    exportgraphics(gcf, ['Results/heatMaps/heatMap_' xVar '_' yVar '.png'])
end