% Make the backwards moving gaits count as fail

load('Results/raw_results.mat')

preSuccess = sum(resultsTable.Success)/length(resultsTable.Success);

idx = resultsTable.fracDesiredVelocity < 0;
resultsTable.xSmooth(idx) = NaN;
resultsTable.pitchSmooth(idx) = NaN;
resultsTable.Success(idx) = 0;

postSuccess = sum(resultsTable.Success)/length(resultsTable.Success);
save('Results/results.mat', 'resultsTable', 'sim', 'gaitParams', 'pNames')