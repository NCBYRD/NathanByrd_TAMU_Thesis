%% RMSE Comparison Bar Chart - Square Trajectory
% 1. Data Setup
rmse_data = [1.701, 3.187;   % Radius = 20
             1.902, 2.133;   % Radius = 40
             2.510, 2.956];  % Radius = 60

% 2. Initialize Figure
fig_rmse = figure('Name', 'RMSE Comparison', 'Color', 'w');
hold on; box on; grid on;

% 3. Create Grouped Bar Chart
b = bar(rmse_data, 'grouped');

% 4. Customize Bar Colors
b(1).FaceColor = [0.2 0.4 0.6]; % Blue for pi/30
b(2).FaceColor = [0.8 0.3 0.3]; % Red for pi/15

% 5. Add Text Labels on top of bars
for i = 1:size(rmse_data, 1)
    for j = 1:size(rmse_data, 2)
        x_pos = b(j).XData(i) + b(j).XOffset;
        y_pos = rmse_data(i, j);
        text(x_pos, y_pos + 0.2, num2str(y_pos, '%.3f'), ...
            'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
    end
end

% 6. Formatting
ylabel('RMSE (mm)', 'FontSize', 10);
xlabel('Radius (mm)', 'FontSize', 10);

% --- FIXED X-AXIS ---
% This forces the ticks to only appear at 1, 2, and 3 but labels them 20, 40, 60
set(gca, 'XTick', 1:3, 'XTickLabel', {'20', '40', '60'});

% Professional Legend
lgd = legend({'\omega = \pi/30 rad/s', '\omega = \pi/15 rad/s'}, ...
             'Location', 'northwest', 'Interpreter', 'tex', 'FontSize', 8);
title(lgd, 'Frequency');

% Set Y-axis limit to give space for labels
ylim([0, max(rmse_data(:)) + 1.5]);

% --- AUTOMATIC EXPORT SETUP (3.5x2.5 inches) ---
set(fig_rmse, 'Units', 'inches');
set(fig_rmse, 'Position', [1, 1, 3.5, 2.5]); 
set(fig_rmse, 'PaperUnits', 'inches');
set(fig_rmse, 'PaperPosition', [0, 0, 3.5, 2.5]);
set(fig_rmse, 'PaperSize', [3.5, 2.5]);

% Ensure font scales well for the small export size
set(gca, 'FontSize', 9);