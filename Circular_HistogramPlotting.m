%% RMSE Comparison Bar Chart
% 1. Data Setup
% Rows represent Phi [0.4, 0.6, 0.8]
% Columns represent Frequency [pi/30 (Slow), pi/15 (Fast)]
rmse_data = [1.288, 1.360;   % Phi = 0.4
             2.014, 2.606;   % Phi = 0.6
             5.049, 5.852]; % Phi = 0.8

% 2. Initialize Figure
fig_rmse = figure('Name', 'RMSE Comparison', 'Color', 'w', 'Position', [200, 200, 700, 500]);
hold on; box on; grid on;

% 3. Create Grouped Bar Chart
b = bar(rmse_data, 'grouped');

% 4. Customize Bar Colors (Optional - matching standard academic colors)
b(1).FaceColor = [0.2 0.4 0.6]; % Blue for pi/30
b(2).FaceColor = [0.8 0.3 0.3]; % Red for pi/15

% 5. Add Text Labels on top of bars
for i = 1:size(rmse_data, 1)
    for j = 1:size(rmse_data, 2)
        x_pos = b(j).XData(i) + b(j).XOffset;
        y_pos = rmse_data(i, j);
        text(x_pos, y_pos + 0.3, num2str(y_pos, '%.3f'), ...
            'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
    end
end

% 6. Formatting
ylabel('RMSE (mm)', 'FontSize', 12);
xlabel('Bending Angle \phi (rad)', 'FontSize', 12);
%title('Tracking Performance: RMSE across Scenarios', 'FontSize', 14);

% Set X-axis tick labels
set(gca, 'XTick', 1:3, 'XTickLabel', {'\phi = 0.4', '\phi = 0.6', '\phi = 0.8'});

% Professional Legend with LaTeX
lgd = legend({'\omega = \pi/30 rad/s', '\omega = \pi/15 rad/s'}, ...
             'Location', 'northwest', 'Interpreter', 'tex');
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