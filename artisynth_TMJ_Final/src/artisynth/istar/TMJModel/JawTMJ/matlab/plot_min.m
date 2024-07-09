% Assuming 'results' is the object returned by bayesopt
% and it contains the history of objective values

% Extract the objective values from the Bayesian optimization results
objectiveValues = results.ObjectiveMinimumTrace;
estimatedObjectiveValues = results.EstimatedObjectiveMinimumTrace;

% Limit to the first 50 iterations
maxIterations = 50;
if length(objectiveValues) > maxIterations
    objectiveValues = objectiveValues(1:maxIterations);
    estimatedObjectiveValues = estimatedObjectiveValues(1:maxIterations);
end

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
hold off;

% Display the final minimum observed and estimated objective values
[minObservedObjective, minObservedIdx] = min(objectiveValues);
[minEstimatedObjective, minEstimatedIdx] = min(estimatedObjectiveValues);
disp(['Final minimum observed objective: ', num2str(minObservedObjective)]);
disp(['Final minimum estimated objective: ', num2str(minEstimatedObjective)]);
