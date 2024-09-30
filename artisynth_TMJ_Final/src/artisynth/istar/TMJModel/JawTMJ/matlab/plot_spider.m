clc; clear; close all;

% Define the ranges for each defect type
zOffsetRange_B = [-2, 5];
leftRollRange_B = [-25, 25];
leftPitchRange_B = [-25, 25];
rightRollRange_B = [-20, 20];
rightPitchRange_B = [-20, 20];

zOffsetRange_S = [-7, 3];
leftRollRange_S = [-10, 10];
leftPitchRange_S = [-15, 15];
rightRollRange_S = [-10, 10];
rightPitchRange_S = [-15, 15];

% Function to map values to [-100, 100] based on range
map_to_percent = @(value, range) ((value - range(1)) / (range(2) - range(1))) * 200 - 100;

% Optimized and Optimized SF=1 values for defect B
optimized_B = [-1.875, 21.582, 23.488, 19.771, 18.563];  % Example values for optimized B
optimizedSF_B = [-2, 25, 25, 20, 20];  % Optimized SF=1 for defect B

% Optimized and Optimized SF=1 values for defect S
optimized_S = [1.816, 22.160, 20.488, 19.819, 19.130];  % Example values for optimized S
optimizedSF_S = [3, 10, -15, -10, -15];  % Optimized SF=1 for defect S

% Map values for defect B
opt_B_mapped = [
    map_to_percent(optimized_B(1), zOffsetRange_B),
    map_to_percent(optimized_B(2), leftRollRange_B),
    map_to_percent(optimized_B(3), leftPitchRange_B),
    map_to_percent(optimized_B(4), rightRollRange_B),
    map_to_percent(optimized_B(5), rightPitchRange_B)
];
optSF_B_mapped = [
    map_to_percent(optimizedSF_B(1), zOffsetRange_B),
    map_to_percent(optimizedSF_B(2), leftRollRange_B),
    map_to_percent(optimizedSF_B(3), leftPitchRange_B),
    map_to_percent(optimizedSF_B(4), rightRollRange_B),
    map_to_percent(optimizedSF_B(5), rightPitchRange_B)
];

% Map values for defect S
opt_S_mapped = [
    map_to_percent(optimized_S(1), zOffsetRange_S),
    map_to_percent(optimized_S(2), leftRollRange_S),
    map_to_percent(optimized_S(3), leftPitchRange_S),
    map_to_percent(optimized_S(4), rightRollRange_S),
    map_to_percent(optimized_S(5), rightPitchRange_S)
];
optSF_S_mapped = [
    map_to_percent(optimizedSF_S(1), zOffsetRange_S),
    map_to_percent(optimizedSF_S(2), leftRollRange_S),
    map_to_percent(optimizedSF_S(3), leftPitchRange_S),
    map_to_percent(optimizedSF_S(4), rightRollRange_S),
    map_to_percent(optimizedSF_S(5), rightPitchRange_S)
];

% Define parameter labels for the radar chart
labels = {'zOffset', 'leftRoll', 'leftPitch', 'rightRoll', 'rightPitch'};

% Convert degrees to radians
theta = linspace(0, 2 * pi, 6);  % 5 parameters -> 6 points (last point closes the radar)

% Add the first point to the end to close the loop
opt_B_mapped = [opt_B_mapped, opt_B_mapped(1)];  % Close the loop for defect B
optSF_B_mapped = [optSF_B_mapped, optSF_B_mapped(1)];  % Close the loop for defect B

opt_S_mapped = [opt_S_mapped, opt_S_mapped(1)];  % Close the loop for defect S
optSF_S_mapped = [optSF_S_mapped, optSF_S_mapped(1)];  % Close the loop for defect S

% Create a figure with two subplots for the radar charts
figure('Position', [100, 100, 1000, 500]);

% Subplot 1: Radar chart for defect B
subplot(1, 2, 1);
hold on;
% Plot optimized values for defect B (ensure both data and theta close the loop)
polarplot(theta, opt_B_mapped, 'r-', 'LineWidth', 2);  % Optimized
polarplot(theta, optSF_B_mapped, 'b--', 'LineWidth', 2);  % Optimized SF=1
set(gca, 'ThetaTick', rad2deg(theta(1:end-1)), 'ThetaTickLabel', labels);  % Set the labels around the radar chart
title('Radar Chart for Defect B');
legend('Optimized', 'Optimized SF=1', 'Location', 'best');
hold off;

% Subplot 2: Radar chart for defect S
subplot(1, 2, 2);
hold on;
% Plot optimized values for defect S
polarplot(theta, opt_S_mapped, 'r-', 'LineWidth', 2);  % Optimized
polarplot(theta, optSF_S_mapped, 'b--', 'LineWidth', 2);  % Optimized SF=1
set(gca, 'ThetaTick', rad2deg(theta(1:end-1)), 'ThetaTickLabel', labels);  % Set the labels around the radar chart
title('Radar Chart for Defect S');
legend('Optimized', 'Optimized SF=1', 'Location', 'best');
hold off;
