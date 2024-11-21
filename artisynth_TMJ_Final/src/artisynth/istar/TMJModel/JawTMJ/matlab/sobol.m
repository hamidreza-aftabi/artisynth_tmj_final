% Sensitivity Analysis Using Surrogate Model and Sobol' Indices

% Clear workspace and load Bayesian optimization results
clear;
load('Sobol_Final_Result_Sensitivity_S_Defect_Trial_2.mat');  % Replace with the correct file name
%load('Final_Result_S_Defect_Trial_1.mat');
% Extract parameter values and output from results
paramValues = results.XTrace;          % Extract parameter values
outputValues = results.ObjectiveTrace;  % Extract objective (output) values

% List of parameter names
paramNames = paramValues.Properties.VariableNames;

% Fit a Gaussian Process (GP) Model to your data
gpModel = fitrgp(paramValues, outputValues, 'BasisFunction', 'none', ...
                 'KernelFunction', 'ardsquaredexponential', 'FitMethod', 'exact');

% Define parameter range for sampling
zOffsetRange = [-2, 5];
leftRollRange = [-25, 25];
leftPitchRange = [-25, 25];
rightRollRange = [-20, 20];
rightPitchRange = [-20, 20];
paramRanges = [zOffsetRange; leftRollRange; leftPitchRange; rightRollRange; rightPitchRange];

% Generate additional samples across the parameter space
numAdditionalSamples = 1000;  % Define number of additional samples
sampledParams = lhsdesign(numAdditionalSamples, size(paramValues, 2));  % Latin Hypercube Sampling
for i = 1:size(paramValues, 2)
    % Scale sampledParams to parameter ranges
    sampledParams(:, i) = paramRanges(i, 1) + ...
        (paramRanges(i, 2) - paramRanges(i, 1)) * sampledParams(:, i);
end

% Predict outputs for the generated samples using the GP model
predictedOutputs = predict(gpModel, sampledParams);

% Perform Sobol' Sensitivity Analysis on the surrogate model outputs
% Initialize Sobol' indices arrays
numParams = size(sampledParams, 2);
firstOrderSobolIndices = zeros(numParams, 1);
totalSobolIndices = zeros(numParams, 1);

% Mean and variance of surrogate model output
meanOutput = mean(predictedOutputs);
varOutput = var(predictedOutputs);

% Loop through each parameter to calculate Sobol' indices
for i = 1:numParams
    % Extract current parameter column
    param_i = sampledParams(:, i);
    
    % Calculate conditional expectation E(Y | X_i) by averaging output values
    % for similar values of param_i. This is an approximation for Sobol indices.
    numBins = 10;  % Number of bins (adjust as needed)
    binEdges = linspace(min(param_i), max(param_i), numBins+1);
    binMeans = zeros(numBins, 1);
    
    for j = 1:numBins
        % Get output values corresponding to bin j for parameter i
        binIdx = param_i >= binEdges(j) & param_i < binEdges(j+1);
        binOutputs = predictedOutputs(binIdx);
        if ~isempty(binOutputs)
            binMeans(j) = mean(binOutputs);
        end
    end
    
    % First-order Sobol index: Var(E(Y | X_i)) / Var(Y)
    firstOrderSobolIndices(i) = var(binMeans) / varOutput;
    
    % Total-effect Sobol index: 1 - Var(Y | X_i) / Var(Y)
    % Approximate Var(Y | X_i) by subtracting first-order effect from total variance
    totalSobolIndices(i) = 1 - (varOutput - var(binMeans)) / varOutput;
end

% Display Sobol' indices
disp('Approximate First-Order Sobol Indices:');
for i = 1:numParams
    fprintf('Parameter %d (%s): %.3f\n', i, paramNames{i}, firstOrderSobolIndices(i));
end

disp('Approximate Total Sobol Indices:');
for i = 1:numParams
    fprintf('Parameter %d (%s): %.3f\n', i, paramNames{i}, totalSobolIndices(i));
end

% Plot Partial Dependence for each parameter as an additional visualization
figure;
for i = 1:numParams
    % Sort parameter values and corresponding outputs for smooth plotting
    [sortedParams, sortIdx] = sort(sampledParams(:, i));
    sortedOutput = predictedOutputs(sortIdx);
    
    % Apply a moving average to smooth the output for clearer visualization
    smoothOutput = movmean(sortedOutput, 5);  % Adjust window size as needed

    % Plot Partial Dependence
    subplot(3, 2, i);
    plot(sortedParams, smoothOutput, '-o');
    title(['Effect of ', paramNames{i}, ' on Surrogate Model Output']);
    xlabel(paramNames{i});
    ylabel('Predicted Output');
end
sgtitle('Partial Dependence Plots');  % Title for entire figure
