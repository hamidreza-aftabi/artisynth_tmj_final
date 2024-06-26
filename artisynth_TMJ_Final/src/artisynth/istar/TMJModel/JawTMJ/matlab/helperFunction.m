function results = helperFunction(results, currentIteration)

% Clear unnecessary variables to manage memory
clearvars -except results currentIteration;

% Define the optimizable variables
zOffsetVar = optimizableVariable('zOffset', [-2, 4.9]);
leftRollVar = optimizableVariable('leftRoll', [-15, 15]);
leftPitchVar = optimizableVariable('leftPitch', [-15, 15]);
rightRollVar = optimizableVariable('rightRoll', [-15, 15]);
rightPitchVar = optimizableVariable('rightPitch', [-15, 15]);

% Combine all optimizable variables into an array
vars = [zOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];

% Debugging output to confirm function execution
disp('Helper function started.');

% Check if we are resuming from a previous session
if exist('results', 'var') && ~isempty(results)
    % Resume Bayesian optimization from previous results
    results = resume(results, ...
        'MaxObjectiveEvaluations', 1);
else
    % Run Bayesian optimization from scratch
    results = bayesopt(@(params) runArtisynthSim(params), vars, ...
        'Verbose', 1, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'MaxObjectiveEvaluations', 1, ... % Initial evaluations
        'GPActiveSetSize', 300, ... % Use active set to speed up Gaussian Process calculations
        'IsObjectiveDeterministic', false, ... % Set if the objective function is deterministic
        'UseParallel', false);
end

% Save the optimization results
save('bayesoptResults.mat', 'results', 'currentIteration');

% Debugging output
disp('Helper function completed.');

end
