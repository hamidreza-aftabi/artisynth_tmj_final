function results = helperFunction(results, currentIteration)

    % Clear unnecessary variables to manage memory
    clearvars -except results currentIteration;

    defectType = 'B';
    trial = 24;

    resultsFile = ['Result_' defectType '_Defect_Trial_' num2str(trial) '.mat'];

    % Define the optimizable variables based on defectType
    if defectType == "B"
        zOffsetVar = optimizableVariable('zOffset', [-2, 5]);
        leftRollVar = optimizableVariable('leftRoll', [-25, 25]);
        leftPitchVar = optimizableVariable('leftPitch', [-25, 25]);
        rightRollVar = optimizableVariable('rightRoll', [-20, 20]);
        rightPitchVar = optimizableVariable('rightPitch', [-20, 20]);
    elseif defectType == "S"
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

    % Set the exploration ratio based on the current iteration
    if currentIteration <= 15
        % Start with full exploration (high ExplorationRatio = 1.0)
        explorationRatio = 1.0;
    elseif currentIteration <= 40
        % Gradually decrease exploration ratio between iteration 16 and 40
        explorationRatio = 1.0 - ((currentIteration - 15) / 25) * (1.0 - 0.5); % Linearly reduce from 1.0 to 0.5
    else
        % After 40 iterations, fix the exploration ratio to 0.5
        explorationRatio = 0.5;
    end

    % If previous results exist, use them to continue the optimization with updated ExplorationRatio
    if exist('results', 'var') && ~isempty(results)
        % Resume Bayesian optimization using the previous results (InitialX and InitialObjective)
        results = bayesopt(@(params) runArtisynthSim(params), vars, ...
            'Verbose', 1, ...
            'AcquisitionFunctionName', 'expected-improvement-plus', ...
            'ExplorationRatio', explorationRatio, ... % Apply updated exploration ratio
            'MaxObjectiveEvaluations', currentIteration, ...
            'GPActiveSetSize', 300, ...
            'IsObjectiveDeterministic', false, ...
            'UseParallel', false, ...
            'InitialX', results.XTrace(1:currentIteration-1,:), ... % Use previously evaluated points
            'InitialObjective', results.ObjectiveTrace(1:currentIteration-1)); % Use previous objectives
    else
        % Run Bayesian optimization from scratch with expected-improvement-plus and varying exploration ratio
        results = bayesopt(@(params) runArtisynthSim(params), vars, ...
            'Verbose', 1, ...
            'AcquisitionFunctionName', 'expected-improvement-plus', ...
            'ExplorationRatio', explorationRatio, ... % Use the current exploration ratio
            'MaxObjectiveEvaluations', currentIteration, ...
            'GPActiveSetSize', 300, ...
            'IsObjectiveDeterministic', false, ...
            'UseParallel', false);
    end

    % Save the optimization results
    save(resultsFile, 'results', 'currentIteration');

    % Debugging output
    disp(['Helper function completed for iteration: ', num2str(currentIteration)]);

end
