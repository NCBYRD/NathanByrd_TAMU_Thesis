%% Unified Error Plotting Script - Task Space Error (Radius vs Time)
% Updated path for Triangle Experiment Data
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\Triangle_ExperimentData\Error_Grouping';

% 1. Identify files
file_list = dir(fullfile(target_path, '*.mat'));
if isempty(file_list)
    error('No files found in: %s. Check the path name.', target_path);
end
colors = lines(length(file_list));

% 2. Initialize Figures
fig_phi = figure('Name', 'Triangle Phi Error', 'Color', 'w');
fig_theta = figure('Name', 'Triangle Theta Error', 'Color', 'w');

% 3. Loop through files
for k = 1:length(file_list)
    filename_str = file_list(k).name;
    data = load(fullfile(target_path, filename_str));
    
    % --- DYNAMIC VARIABLE CHECK ---
    if isfield(data, 'val_plot1_filtered')
        val_data = data.val_plot1_filtered;
    elseif isfield(data, 'val_plot1')
        val_data = data.val_plot1;
    else
        val_data = [];
    end
    
    if isfield(data, 't_plot') && ~isempty(val_data)
        t = data.t_plot(:); 
        
        % Ensure orientation is Nx2
        if size(val_data, 1) == 2 && size(val_data, 2) ~= 2
            val_data = val_data'; 
        end
        
        num_pts = min(length(t), size(val_data, 1));
        phi_error   = real(val_data(1:num_pts, 1)); 
        theta_error = real(val_data(1:num_pts, 2)); 
        t_actual    = t(1:num_pts);
        
        % --- ROBUST FILENAME PARSING ---
        % Extracts decimal (e.g., 0.05) and converts to mm
        tokens = regexp(filename_str, '(\d+\.\d+)Length', 'tokens');
        if ~isempty(tokens)
            rad_val = num2str(str2double(tokens{1}{1}) * 1000); 
        else
            % Fallback for standard integers if decimal point is missing
            tokens_int = regexp(filename_str, '(\d+)Length', 'tokens');
            if ~isempty(tokens_int)
                rad_val = tokens_int{1}{1};
            else
                rad_val = '??';
            end
        end
        
        % Frequency mapping based on your naming convention
        freq_label = '';
        if contains(filename_str, 'Time30'),     freq_label = '\pi/15';
        elseif contains(filename_str, 'Time60'), freq_label = '\pi/30';
        end
        
        % Build clean LaTeX legend string
        if ~isempty(freq_label) && ~strcmp(rad_val, '??')
            name_str = sprintf('R_{a,%s} = %s mm', freq_label, rad_val);
        else
            name_str = strrep(filename_str, '_', ' '); 
        end
        
        % Plotting
        figure(fig_phi); hold on; grid on; box on;
        plot(t_actual, phi_error, 'LineWidth', 1.2, 'Color', colors(k, :), 'DisplayName', name_str);
        
        figure(fig_theta); hold on; grid on; box on;
        plot(t_actual, theta_error, 'LineWidth', 1.2, 'Color', colors(k, :), 'DisplayName', name_str);
    end
end

%% 4. Final Formatting & Automatic Export (3.5x2.5)
figs = [fig_phi, fig_theta];
y_labels = {'Bending Angle Error \phi (rad)', 'Rotational Error \theta (rad)'};

for i = 1:length(figs)
    curr_fig = figs(i);
    figure(curr_fig); 
    drawnow; 

    ax = gca;
    xlabel('Time (s)', 'FontSize', 10);
    ylabel(y_labels{i}, 'FontSize', 10);
    
    % --- FIXED X-AXIS LIMITS ---
    if ~isempty(ax.Children)
        set(ax, 'XLim', [0 120]);
    end
    
    % --- LEGEND SETUP (TOP, CENTERED OUTSIDE, 3 COLUMNS) ---
    lgd = legend('show');
    if ~isempty(lgd)
        set(lgd, 'Interpreter', 'tex', ...
                 'Location', 'northoutside', ... % Centers it ABOVE the plot area
                 'Orientation', 'horizontal', ... 
                 'NumColumns', 3, ...          
                 'IconColumnWidth', 5, ...     % Keep icons compact
                 'EdgeColor', 'k', ...         
                 'FontSize', 7);               
    end
    
    % --- AUTOMATIC EXPORT SETUP (3.5" x 2.5") ---
    set(curr_fig, 'Units', 'inches');
    set(curr_fig, 'Position', [1, 1, 3.5, 2.5]); 
    
    % Adjust Axes Position to give the legend room to breathe at the top
    % [left bottom width height] - normalized units
    set(ax, 'Units', 'normalized');
    set(ax, 'Position', [0.15, 0.18, 0.75, 0.60]); 
    
    set(curr_fig, 'PaperUnits', 'inches');
    set(curr_fig, 'PaperPosition', [0, 0, 3.5, 2.5]);
    set(curr_fig, 'PaperSize', [3.5, 2.5]);
    
    % Final font settings
    set(ax, 'FontSize', 9, 'FontName', 'Helvetica');
end