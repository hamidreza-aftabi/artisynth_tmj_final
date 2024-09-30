clc; clear; close all;

% Load the three results
load('Result_B_Defect_Trial_26.mat');
minObjectiveTrace_26 = results.ObjectiveMinimumTrace;  % Extracting ObjectiveMinimumTrace from results

load('Result_B_Defect_Trial_20.mat');
minObjectiveTrace_20 = results.ObjectiveMinimumTrace;  % Extracting ObjectiveMinimumTrace from results

load('Result_B_Defect_Trial_21.mat');
minObjectiveTrace_21 = results.ObjectiveMinimumTrace;  % Extracting ObjectiveMinimumTrace from results

% Find the length of the shortest result to ensure equal comparison
min_length = min([length(minObjectiveTrace_26), length(minObjectiveTrace_20), length(minObjectiveTrace_21)]);

% Truncate all results to the same length
minObjectiveTrace_26 = minObjectiveTrace_26(1:min_length);
minObjectiveTrace_20 = minObjectiveTrace_20(1:min_length);
minObjectiveTrace_21 = minObjectiveTrace_21(1:min_length);

% Combine the traces into a matrix for easy calculation
minObjectiveTraces = [minObjectiveTrace_26, minObjectiveTrace_20, minObjectiveTrace_21];

% Calculate the average and standard deviation of the minimum objectives at each iteration
average_min_objective = mean(minObjectiveTraces, 2);
std_min_objective = std(minObjectiveTraces, 0, 2);

% Define iterations
iterations = 1:min_length;

% Plot the average minimum objective vs iterations
figure;
hold on;

% Plot the shaded area for standard deviation (average ± std) in light purple color
h1 = fill([iterations, fliplr(iterations)], ...
     [average_min_objective' + std_min_objective', fliplr(average_min_objective' - std_min_objective')], ...
     [0.9, 0.8, 0.9], 'EdgeColor', 'none');  % Light purple shaded area

% Plot the average line in purple
h2 = plot(iterations, average_min_objective, 'Color', [0.5, 0, 0.5], 'LineWidth', 2); % Purple

% Add labels and title
xlabel('Iterations');
ylabel('Average Minimum Objective');
title('Average Minimum Objective vs Iterations with Standard Deviation');
grid on;

% Add legend for the main line and the shaded area
legend([h2, h1], {'Average Minimum Objective', 'Standard Deviation'}, 'Location', 'best');

hold off;
