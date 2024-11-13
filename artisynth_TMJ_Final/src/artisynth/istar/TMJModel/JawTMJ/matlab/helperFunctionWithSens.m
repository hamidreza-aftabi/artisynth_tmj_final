function results = helperFunctionWithSens(results, currentIteration)

    % Clear unnecessary variables to manage memory
    clearvars -except results currentIteration;

    defectType = 'B';  % Set defect type ('B' or 'S')
    trial = 1;

    resultsFile = ['Sens_Final_Result_Sensitivity_' defectType '_Defect_Trial_' num2str(trial) '.mat'];

    % Define the range of variables and initial points based on defectType
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



    % Define the optimizable variables
    zOffsetVar = optimizableVariable('zOffset', zOffsetRange);
    leftRollVar = optimizableVariable('leftRoll', leftRollRange);
    leftPitchVar = optimizableVariable('leftPitch', leftPitchRange);
    rightRollVar = optimizableVariable('rightRoll', rightRollRange);
    rightPitchVar = optimizableVariable('rightPitch', rightPitchRange);


    % Combine all optimizable variables into an array
    vars = [zOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];
    %vars = [zOffsetVar, leftRollVar, leftPitchVar, rightRollVar, rightPitchVar];


    % Check if we are resuming from a previous session
    if exist('results', 'var') && ~isempty(results)
        % Resume Bayesian optimization from previous results
        results = resume(results, ...
            'MaxObjectiveEvaluations', 1);
    else
        % Run Bayesian optimization with the combined initial points
        results = bayesopt(@(params) runArtisynthSimWithSens(params), vars, ...
            'Verbose', 1, ...
            'AcquisitionFunctionName', 'expected-improvement', ...
            'MaxObjectiveEvaluations', 1, ... % Initial evaluations
            'IsObjectiveDeterministic', false, ...
            'UseParallel', false);
    end

    % Save the optimization results
    save(resultsFile, 'results', 'currentIteration');

    % Debugging output
    disp('Helper function completed.');

end
