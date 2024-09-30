function results = helperFunctionV1(results, currentIteration)

% Clear unnecessary variables to manage memory
clearvars -except results currentIteration;

defectType = 'B';
trial = 30;


resultsFile = ['Result_Safety_1' defectType '_Defect_Trial_' num2str(trial) '.mat'];

if defectType == "B"

    % Define the optimizable variables
    zOffsetVar = optimizableVariable('zOffset', [-2, 5]);
    leftRollVar = optimizableVariable('leftRoll', [-25, 25]);
    leftPitchVar = optimizableVariable('leftPitch', [-25, 25]);
    rightRollVar = optimizableVariable('rightRoll', [-20, 20]);
    rightPitchVar = optimizableVariable('rightPitch', [-20, 20]);


elseif  defectType == "S"
    
     % Define the optimizable variables
    zOffsetVar = optimizableVariable('zOffset', [-7, 3]);
    leftRollVar = optimizableVariable('leftRoll', [-10, 10]);
    leftPitchVar = optimizableVariable('leftPitch', [-15, 15]);
    rightRollVar = optimizableVariable('rightRoll', [-10, 10]);
    rightPitchVar = optimizableVariable('rightPitch', [-15, 15]);


end


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
        'AcquisitionFunctionName', 'expected-improvement', ...
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
