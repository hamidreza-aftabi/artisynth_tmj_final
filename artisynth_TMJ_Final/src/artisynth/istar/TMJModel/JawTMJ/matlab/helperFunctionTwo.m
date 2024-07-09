function results = helperFunctionTwo(results, currentIteration)


% Clear unnecessary variables to manage memory
clearvars -except results currentIteration;

resultsFile = 'bayesoptResults_25_TMJ_FIXED_Trial7_Costhalf.mat';


% Define the optimizable variables
zOffsetVar = optimizableVariable('zOffset', [-2, 4.9]);
leftRollVar = optimizableVariable('leftRoll', [-30, 30]);
leftPitchVar = optimizableVariable('leftPitch', [-30, 30]);
rightRollVar = optimizableVariable('rightRoll', [-15, 15]);
rightPitchVar = optimizableVariable('rightPitch', [-15, 15]);

% Combine all optimizable variables into an array
vars = [zOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];

% Debugging output to confirm function execution
disp('Helper function started.');

rng('shuffle');

% Check if we are resuming from a previous session
if exist('results', 'var') && ~isempty(results)
    % Resume Bayesian optimization from previous results
    results = resume(results, ...
        'MaxObjectiveEvaluations', 1);
else
    % Run Bayesian optimization from scratch
    results = bayesopt(@(params) runArtisynthSimTwo(params), vars, ...
        'Verbose', 1, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
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
