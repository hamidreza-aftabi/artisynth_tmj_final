% Assuming 'results' is the output from bayesopt and contains the history of objective values

% Number of iterations
numIterations = length(results.ObjectiveMinimumTrace);

% Extract the minimum objective values at each iteration
objectiveValues = results.ObjectiveMinimumTrace;

% Extract the points evaluated at each iteration
evaluatedPoints = results.XTrace;

% Initialize vectors to store mean and standard deviation of predicted objective values
predictedMeans = zeros(1, numIterations);
predictedStds = zeros(1, numIterations);

% Ensure the variable names are consistent
variableNames = evaluatedPoints.Properties.VariableNames;

% Calculate the predicted mean and standard deviation at each evaluated point using the GP model
for i = 1:numIterations
    % Ensure the input is a table
    pointTable = array2table(evaluatedPoints{i, :}, 'VariableNames', variableNames);
    
    % Predict the mean and standard deviation
    [predictedMeans(i), predictedStds(i)] = predictObjective(results, pointTable);
end

% Calculate 95% confidence intervals
confidenceLevel = 0.95;
z = norminv(1 - (1 - confidenceLevel) / 2); % z-value for 95% confidence
confidenceIntervals = z * predictedStds;

% Number of iterations
iterations = 1:numIterations;

% Plot the minimum observed objective values with confidence intervals
figure;
hold on;

% Use custom colors
errorColor = [0.8500 0.3250 0.0980]; % RGB color for the error bars (pink)
pointColor = [0 0.4470 0.7410]; % RGB color for the data points (blue)

% Plot the error bars
errorbar(iterations, objectiveValues, confidenceIntervals, 'LineStyle', 'none', 'LineWidth', 1.5, 'Color', errorColor);

% Plot the data points and connect them with a line
plot(iterations, objectiveValues, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', pointColor, 'MarkerEdgeColor', pointColor, 'MarkerFaceColor', pointColor);

% Annotate the minimum observed point
[minObservedObjective, minObservedIdx] = min(objectiveValues);
plot(minObservedIdx, minObservedObjective, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(minObservedIdx, minObservedObjective, sprintf('  Min Observed: %.4f', minObservedObjective), ...
    'VerticalAlignment', 'top', 'Position', [minObservedIdx, minObservedObjective + 0.1 * abs(minObservedObjective)], 'FontSize', 10, 'Color', 'r');

% Add titles and labels with larger fonts
title('Minimum Observed Objective with Confidence Intervals', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Iteration', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Objective Value', 'FontSize', 12, 'FontWeight', 'bold');

% Add grid and legend
grid on;
legend('Confidence Interval', 'Observed Objective', 'Location', 'best', 'FontSize', 12);

% Display the final minimum observed objective values with confidence intervals
finalMinObservedObjective = objectiveValues(end);
finalConfidenceInterval = confidenceIntervals(end);
disp(['Final minimum observed objective: ', num2str(finalMinObservedObjective)]);
disp(['95% confidence interval for the final minimum observed objective: [', ...
    num2str(finalMinObservedObjective - finalConfidenceInterval), ', ', ...
    num2str(finalMinObservedObjective + finalConfidenceInterval), ']']);

hold off;
