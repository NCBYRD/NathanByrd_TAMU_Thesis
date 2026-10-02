%% Unified Multi-File Trajectory Plotting Script - Triangle Trajectory Overlay
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\Triangle_ExperimentData\NDI_Grouping';

% 1. Constants
radii_targets = [50, 60, 70]; 

% 2. Identify files
file_list = dir(fullfile(target_path, '*.mat'));
if isempty(file_list), error('No files found in path.'); end

% 3. Initialize Figure
fig_overlay = figure('Name', 'NDI Overlay vs Desired Triangle Trajectories', 'Color', 'w');
hold on; grid on; box on;
colors = lines(length(file_list));

% 4. Plot Desired Reference Triangles
% Coordinates for an equilateral triangle centered at (0,0)
for r = 1:length(radii_targets)
    R = radii_targets(r);
    
    % Vertices for an equilateral triangle pointing upward (Flipped Fix)
    % Starting at 3*pi/2 (270 degrees) puts the base at the bottom
    th = linspace(3*pi/2, 3*pi/2 + 2*pi, 4); 
    tri_x = R * cos(th);
    tri_y = R * sin(th);
    
    plot(tri_x, tri_y, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, ...
        'DisplayName', sprintf('R_{d} = %d', R));
end

% 5. Loop through files and overlay data
for k = 1:length(file_list)
    filename_str = file_list(k).name;
    data = load(fullfile(target_path, filename_str));
    
    % Data Variable Selection
    if isfield(data, 'x_actual')
        x_p = data.x_actual; y_p = data.y_actual;
    elseif isfield(data, 'x_final')
        x_p = data.x_final;  y_p = data.y_final;
    elseif isfield(data, 'val_plot1_filtered') 
        x_p = data.val_plot1_filtered(:, 1); 
        y_p = data.val_plot1_filtered(:, 2);
    else
        continue; 
    end
    
    % --- ROBUST FILENAME PARSING ---
    % Automatically extracts 0.05, 0.06, 0.07 and converts to mm
    tokens = regexp(filename_str, '(\d+\.\d+)Length', 'tokens');
    if ~isempty(tokens)
        rad_val = num2str(str2double(tokens{1}{1}) * 1000); 
    else
        rad_val = '??';
    end
    
    freq_val = '';
    if contains(filename_str, 'Time30'),     freq_val = '\pi/15';
    elseif contains(filename_str, 'Time60'), freq_val = '\pi/30'; 
    end
    
    if ~strcmp(rad_val, '??') && ~isempty(freq_val)
        streamlined_name = sprintf('R_{a,%s} = %s', freq_val, rad_val);
    else
        streamlined_name = strrep(filename_str, '_', ' '); 
    end
    
    plot(x_p, y_p, 'LineWidth', 1.2, 'Color', colors(k, :), ...
         'DisplayName', streamlined_name); 
end

% 6. Standard Formatting
xlabel('Relative X (mm)', 'FontSize', 10);
ylabel('Relative Y (mm)', 'FontSize', 10);
axis equal; 
view(2);

% Legend placement (matches Square_XYPath style)
lgd = legend('show');
set(lgd, 'Interpreter', 'tex', 'Location', 'eastoutside', 'NumColumns', 1, 'FontSize', 8);

% --- AUTOMATIC EXPORT SETUP (3.5x2.5) ---
set(fig_overlay, 'Units', 'inches');
set(fig_overlay, 'Position', [1, 1, 3.5, 2.5]); 
set(fig_overlay, 'PaperUnits', 'inches');
set(fig_overlay, 'PaperPosition', [0, 0, 3.5, 2.5]);
set(fig_overlay, 'PaperSize', [3.5, 2.5]);
set(gca, 'FontSize', 9, 'FontName', 'Helvetica');