% Assuming 'results' is the object returned by bayesopt
% and it contains the history of objective values

% Extract the objective values from the Bayesian optimization results
objectiveValues = results.ObjectiveMinimumTrace;
estimatedObjectiveValues = results.EstimatedObjectiveMinimumTrace;

% Number of iterations
iterations = 1:length(objectiveValues);

% Plot the minimum observed and estimated objective versus iteration
figure;
plot(iterations, objectiveValues, '-o', 'LineWidth', 2, 'MarkerSize', 6);
hold on;
plot(iterations, estimatedObjectiveValues, '-x', 'LineWidth', 2, 'MarkerSize', 6);
title('Minimum Observed and Estimated Objective vs. Iteration');
xlabel('Iteration');
ylabel('Objective Value');
legend('Observed Objective', 'Estimated Objective');
grid on;

% Annotate the minimum observed point
[minObservedObjective, minObservedIdx] = min(objectiveValues);
plot(minObservedIdx, minObservedObjective, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(minObservedIdx, minObservedObjective, sprintf('  Min Observed: %.4f', minObservedObjective), ...
    'VerticalAlignment', 'top', 'Position', [minObservedIdx, minObservedObjective + 0.1 * abs(minObservedObjective), 0], 'FontSize', 10);

% Annotate the minimum estimated point
[minEstimatedObjective, minEstimatedIdx] = min(estimatedObjectiveValues);
plot(minEstimatedIdx, minEstimatedObjective, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(minEstimatedIdx, minEstimatedObjective, sprintf('  Min Estimated: %.4f', minEstimatedObjective), ...
    'VerticalAlignment', 'bottom', 'Position', [minEstimatedIdx, minEstimatedObjective - 0.1 * abs(minEstimatedObjective), 0], 'FontSize', 10);

hold off;

% Display the final minimum observed and estimated objective values
disp(['Final minimum observed objective: ', num2str(minObservedObjective)]);
disp(['Final minimum estimated objective: ', num2str(minEstimatedObjective)]);
