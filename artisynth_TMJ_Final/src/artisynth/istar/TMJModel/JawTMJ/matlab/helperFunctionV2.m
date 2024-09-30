function results = helperFunctionV2(results, currentIteration)

% Clear unnecessary variables to manage memory
clearvars -except results currentIteration;

defectType = 'B';
trial = 31;

resultsFile = ['Result_Safety_1' defectType '_Defect_Trial_' num2str(trial) '.mat'];

% Define the range of variables based on defectType
if defectType == "B"
    zOffsetRange = [-2, 5];
    leftRollRange = [-25, 25];
    leftPitchRange = [-25, 25];
    rightRollRange = [-20, 20];
    rightPitchRange = [-20, 20];

elseif defectType == "S"
    zOffsetRange = [-7, 3];
    leftRollRange = [-10, 10];
    leftPitchRange = [-15, 15];
    rightRollRange = [-10, 10];
    rightPitchRange = [-15, 15];
end

% Get the midpoints and ranges
midpoints = [mean(zOffsetRange), mean(leftRollRange), mean(leftPitchRange), mean(rightRollRange), mean(rightPitchRange)];
ranges = [diff(zOffsetRange), diff(leftRollRange), diff(leftPitchRange), diff(rightRollRange), diff(rightPitchRange)] / 2;

% Generate all 32 sign combinations (2^5 = 32)
signCombinations = dec2bin(0:31) - '0';  % Binary matrix for sign combinations
signCombinations(signCombinations == 0) = -1;  % Convert 0 to -1 for negative signs

% Apply the sign combinations to the midpoints
initialPoints = midpoints + signCombinations .* ranges;


% Convert initial points to a table for Bayesian optimization
varsNames = {'zOffset', 'leftRoll', 'leftPitch', 'rightRoll', 'rightPitch'};
initialTable = array2table(initialPoints, 'VariableNames', varsNames);


% Debugging output to confirm function execution
disp('Helper function started with all sign combinations.');

% Define the optimizable variables
zOffsetVar = optimizableVariable('zOffset', zOffsetRange);
leftRollVar = optimizableVariable('leftRoll', leftRollRange);
leftPitchVar = optimizableVariable('leftPitch', leftPitchRange);
rightRollVar = optimizableVariable('rightRoll', rightRollRange);
rightPitchVar = optimizableVariable('rightPitch', rightPitchRange);

% Combine all optimizable variables into an array
vars = [zOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];

% Check if we are resuming from a previous session
if exist('results', 'var') && ~isempty(results)
    % Resume Bayesian optimization from previous results
    results = resume(results, ...
        'MaxObjectiveEvaluations', 1);
else
    % Run Bayesian optimization with all sign combinations as initial points
    results = bayesopt(@(params) runArtisynthSim(params), vars, ...
        'Verbose', 1, ...
        'AcquisitionFunctionName', 'expected-improvement', ...
        'InitialX', initialTable, ...  % Provide the generated initial points
        'NumSeedPoints', 15, ...
        'MaxObjectiveEvaluations', 1, ... % Initial evaluations
        'GPActiveSetSize', 300, ... % Use active set to speed up Gaussian Process calculations
        'IsObjectiveDeterministic', false, ... % Set if the objective function is deterministic
        'UseParallel', false);
end

% Save the optimization results
save(resultsFile, 'results', 'currentIteration');

% Debugging output
disp('Helper function completed.');

end
