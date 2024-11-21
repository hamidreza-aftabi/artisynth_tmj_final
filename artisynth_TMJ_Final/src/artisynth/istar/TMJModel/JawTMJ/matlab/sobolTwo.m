% Sensitivity Analysis Using Surrogate Model and Sobol' Indices for 6 Parameters

% Clear workspace and load Bayesian optimization results
clear;
load('Final_Result_RB_Defect_Trial_2.mat');  % Load your actual results file

% Extract parameter values and output from results
paramValues = results.XTrace;          % Extract parameter values
outputValues = results.ObjectiveTrace;  % Extract objective (output) values

% Skip row 23 in both paramValues and outputValues
paramValues(23, :) = [];
outputValues(23) = [];

% List of parameter names
paramNames = {'zOffset', 'rdpOffset', 'leftRoll', 'leftPitch', 'rightRoll', 'rightPitch'};

% Fit a Gaussian Process (GP) Model to your data
gpModel = fitrgp(paramValues, outputValues, 'BasisFunction', 'none', ...
                 'KernelFunction', 'ardsquaredexponential', 'FitMethod', 'exact');

% Define parameter range for sampling
zOffsetRange = [-5, 5];
rdpOffsetRange = [-7, 7];
leftRollRange = [-25, 25];
leftPitchRange = [-25, 25];
rightRollRange = [-15, 15];
rightPitchRange = [-15, 15];
paramRanges = [zOffsetRange; rdpOffsetRange; leftRollRange; leftPitchRange; rightRollRange; rightPitchRange];

% Generate additional samples across the parameter space
numAdditionalSamples = 1000;  % Define number of additional samples
numVariables = length(paramNames); % Number of parameters (6 in this case)

% Use Latin Hypercube Sampling to generate samples
lhsSamples = lhsdesign(numAdditionalSamples, numVariables);

% Scale the LHS samples to the actual variable ranges
scaledSamples = bsxfun(@plus, paramRanges(:, 1)', bsxfun(@times, lhsSamples, diff(paramRanges, [], 2)'));

% Predict outputs for the generated samples using the GP model
predictedOutputs = predict(gpModel, scaledSamples);

% Perform Sobol' Sensitivity Analysis on the surrogate model outputs
% Initialize Sobol' indices arrays
firstOrderSobolIndices = zeros(numVariables, 1);
totalSobolIndices = zeros(numVariables, 1);

% Mean and variance of surrogate model output
meanOutput = mean(predictedOutputs);
varOutput = var(predictedOutputs);

% Binning parameters for Sobol' approximation
numBins = 20;  % Number of bins for each parameter

% Loop through each parameter to calculate Sobol' indices
for i = 1:numVariables
    % Extract current parameter column
    param_i = scaledSamples(:, i);
    
    % Calculate conditional expectation E(Y | X_i) by averaging output values
    % for similar values of param_i. This is an approximation for Sobol indices.
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
for i = 1:numVariables
    fprintf('%s: %.3f\n', paramNames{i}, firstOrderSobolIndices(i));
end

disp('Approximate Total Sobol Indices:');
for i = 1:numVariables
    fprintf('%s: %.3f\n', paramNames{i}, totalSobolIndices(i));
end

% Plot Partial Dependence for each parameter as an additional visualization
figure;
for i = 1:numVariables
    % Sort parameter values and corresponding outputs for smooth plotting
    [sortedParams, sortIdx] = sort(scaledSamples(:, i));
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
