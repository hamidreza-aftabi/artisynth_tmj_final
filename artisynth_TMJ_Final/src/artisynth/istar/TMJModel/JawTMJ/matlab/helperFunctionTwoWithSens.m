function results = helperFunctionTwoWithSens(results, currentIteration)


    % Clear unnecessary variables to manage memory
    clearvars -except results currentIteration;

    defectType = 'RB'; 
    trial = 4;

    resultsFile = ['Final_Result_' defectType '_Defect_Trial_' num2str(trial) '.mat'];

    % Define the range of variables and initial points based on defectType
    
    zOffsetRange = [-5, 5];
    rdpOffsetRange = [-7, 7]; 
    leftRollRange = [-25, 25];
    leftPitchRange = [-25, 25];
    rightRollRange = [-15, 15];
    rightPitchRange = [-15, 15];


    % Define the optimizable variables
    zOffsetVar = optimizableVariable('zOffset', zOffsetRange);
    rdpOffsetVar = optimizableVariable ('rdpOffset', rdpOffsetRange);
    leftRollVar = optimizableVariable('leftRoll', leftRollRange);
    leftPitchVar = optimizableVariable('leftPitch', leftPitchRange);
    rightRollVar = optimizableVariable('rightRoll', rightRollRange);
    rightPitchVar = optimizableVariable('rightPitch', rightPitchRange);

    % Combine all optimizable variables into an array
    vars = [zOffsetVar, rdpOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];

    % Check if we are resuming from a previous session
    if exist('results', 'var') && ~isempty(results)
        % Resume Bayesian optimization from previous results
        results = resume(results, ...
            'MaxObjectiveEvaluations', 1);
    else
        % Run Bayesian optimization with the combined initial points
        results = bayesopt(@(params) runArtisynthSimTwoWithSens(params), vars, ...
            'Verbose', 1, ...
            'AcquisitionFunctionName', 'expected-improvement-plus', ...
            'ExplorationRatio', 0.6, ...
            'InitialX', initialTable, ...  % Provide the combined initial points
            'MaxObjectiveEvaluations', 1, ... % Initial evaluations
            'IsObjectiveDeterministic', false, ...
            'UseParallel', false);
    end

    % Save the optimization results
    save(resultsFile, 'results', 'currentIteration');

    % Debugging output
    disp('Helper function completed.');

end
