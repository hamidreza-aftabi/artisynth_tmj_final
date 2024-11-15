function results = helperFunctionV3(results, currentIteration)

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
        initialPoints = [-1.82, 24.2, 20.4, 19.98, 18.7];  % Initial point for defect 'B'


    elseif defectType == "S"
        zOffsetRange = [-7, 3];
        leftRollRange = [-10, 10];
        leftPitchRange = [-15, 15];
        rightRollRange = [-10, 10];
        rightPitchRange = [-15, 15];
        initialPoints = [-7 0 0 0 0];  % Initial point for defect 'S'
    end

    % Use Latin Hypercube Sampling to generate 19 new sample points (20 total with initial point)
    numSamples = 3;  % Number of additional samples to generate (19 + 1 initial point = 20)
    numVariables = 5; % Number of variables to sample

    % Define the variable ranges
    varRanges = [zOffsetRange; leftRollRange; leftPitchRange; rightRollRange; rightPitchRange];

    % Set the random seed for reproducibility (optional)
    %rng(42);

    % Generate Latin Hypercube Samples in the range [0,1]
    lhsSamples = lhsdesign(numSamples, numVariables);

    % Scale the LHS samples to the actual variable ranges
    scaledSamples = bsxfun(@plus, varRanges(:, 1)', bsxfun(@times, lhsSamples, diff(varRanges, [], 2)'));

    % Convert the LHS samples to a table for Bayesian optimization
    varsNames = {'zOffset', 'leftRoll', 'leftPitch', 'rightRoll', 'rightPitch'};
    additionalTable = array2table(scaledSamples, 'VariableNames', varsNames);

    % Convert the initial points to a table
    initialTablePoint = array2table(initialPoints, 'VariableNames', varsNames);

    % Combine the initial point with the LHS-generated points
    initialTable = [initialTablePoint; additionalTable];  % Concatenating tables

    % Debugging output to confirm function execution
    disp(['Helper function started with defect type ' defectType ' and LHS sampling with 20 total points.']);

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
        % Run Bayesian optimization with the combined initial points
        results = bayesopt(@(params) runArtisynthSim(params), vars, ...
            'Verbose', 1, ...
            'AcquisitionFunctionName', 'expected-improvement', ...
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
