

% Extract time and values for best result (left)
timeBestLeft = left_percent(:, 1);
valuesBestLeft = left_percent(:, 2);

% Extract time and values for typical result (left)
timeTypicalLeft = typ_left_percent(:, 1);
valuesTypicalLeft = typ_left_percent(:, 2);

% Plot the percent left for best and typical result
figure;
subplot(2, 1, 1); % Create a subplot for left percent
plot(timeBestLeft, valuesBestLeft, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Optimized');
hold on;
plot(timeTypicalLeft, valuesTypicalLeft, '-x', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Initial Guess');
title('Left Plane Apposition Percentage: Optimized vs Initial Guess');
xlabel('Time (Chewing Cycle)');
ylabel('Apposition Percentage (Left Plane)');
legend('show');
grid on;
hold off;



% Extract time and values for best result (right)
timeBestRight = right_percent(:, 1);
valuesBestRight = right_percent(:, 2);

% Extract time and values for typical result (right)
timeTypicalRight = typ_right_percent(:, 1);
valuesTypicalRight = typ_right_percent(:, 2);

% Plot the percent right for best and typical result
subplot(2, 1, 2); % Create a subplot for right percent
plot(timeBestRight, valuesBestRight, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Optimized');
hold on;
plot(timeTypicalRight, valuesTypicalRight, '-x', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Initial Guess');
title('Right Plane Apposition Percentage: Optimized vs Initial Guess');
xlabel('Time (Chewing Cycle)');
ylabel('Apposition Percentage (Right Plane)');
legend('show');
grid on;
hold off;
