clc; clear; close all;

% Data for the parameters
values = [
    -1.875, 21.582, 23.488, 19.771, 18.563;
    -1.816, 22.160, 21.489, 19.820, 19.130;
    -2.000, 25.000, 25.000, 20.000, 20.000
];

% Calculate mean and standard deviation
mean_values = mean(values);
std_dev = std(values);

% Labels for the parameters
labels = {'leftRoll (°)', 'leftPitch (°)', 'rightRoll (°)', 'rightPitch (°)', 'zOffset (mm)'};

% Extract zOffset and degree parameters
zOffset = mean_values(1);
deg_params = mean_values(2:5);

% Standard deviations for zOffset and degree parameters
std_zOffset = std_dev(1);
std_deg_params = std_dev(2:5);

% Color palette inspired by the image
blue_color = [0.2, 0.6, 1];  % Soft blue
dark_blue_color = [0, 0.3, 0.7];  % Darker blue for axis
pink_color = [1, 0.6, 0.8];  % Soft pink
dark_pink_color = [0.7, 0, 0.4];  % Darker pink for axis

% Bar plot configuration
figure('Position', [100, 100, 800, 450], 'Color', 'w');  % Slightly reduced width for a more compact look

% Increased spacing between degree parameters by shifting x positions
x_degrees = [1, 2.5, 4, 5.5];  % Increased gaps between bars
x_zOffset = 8;  % Distance for zOffset (a bit further to create a visual gap)

% Plot degree parameters on the left y-axis
yyaxis left;
h1 = bar(x_degrees, deg_params, 0.4, 'FaceColor', blue_color, 'EdgeColor', 'none');  % Blue bars, narrower bar width
hold on;
errorbar(x_degrees, deg_params, std_deg_params, 'LineStyle', 'none', 'LineWidth', 1.5, 'Color', dark_blue_color);  % Error bars in dark blue
ylabel('Degrees (°)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([15, ceil(max(deg_params + std_deg_params)) + 5]);
yticks(15:5:ceil(max(deg_params + std_deg_params)) + 5);

% Plot zOffset on the right y-axis
yyaxis right;
h2 = bar(x_zOffset, zOffset, 0.5, 'FaceColor', pink_color, 'EdgeColor', 'none');  % Pink bar, same narrow width
errorbar(x_zOffset, zOffset, std_zOffset, 'LineStyle', 'none', 'LineWidth', 1.5, 'Color', dark_pink_color);  % Error bars in dark pink
ylabel('zOffset (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([floor(zOffset - std_zOffset - 5), ceil(zOffset + std_zOffset + 5)]);
yticks(floor(zOffset - std_zOffset - 5):2:ceil(zOffset + std_zOffset + 5));

% Customize x-axis
set(gca, 'XTick', [x_degrees, x_zOffset], 'XTickLabel', labels, 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Parameters', 'FontSize', 13, 'FontWeight', 'bold');

% Customize plot appearance
ax = gca;
ax.YGrid = 'off';   % No grid lines
ax.XGrid = 'off';   % No grid lines

% Darker axis colors
ax.YColor = dark_blue_color;    % Left axis in dark blue
ax.YAxis(2).Color = dark_pink_color;  % Right axis in dark pink

% Add clean legend with no borders
legend([h1, h2], {'Degree Parameters', 'zOffset (mm)'}, 'Location', 'northwest', 'Box', 'off', 'FontSize', 11);

% Remove box outline from the plot
set(gca, 'box', 'off');

% Add a subtle title and improve visual style
title('Comparison of Degree Parameters and zOffset', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);

% Final touch to make the plot visually balanced
hold off;
