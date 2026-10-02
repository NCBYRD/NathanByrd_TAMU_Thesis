%% Unified Task-Space Script: Phi and Theta vs Time (Triangle Trajectory)
% Path updated for Triangle Experiment Data
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\Triangle_ExperimentData\TaskSpace_Grouping';

file_list = dir(fullfile(target_path, '*.mat'));
if isempty(file_list), error('No files found in path.'); end
colors = lines(length(file_list));

% Initialize Figures
fig_phi = figure('Name', 'Triangle Phi Tracking', 'Color', 'w'); 
fig_theta = figure('Name', 'Triangle Theta Tracking', 'Color', 'w'); 

for k = 1:length(file_list)
    filename_str = file_list(k).name;
    data = load(fullfile(target_path, filename_str));
    
    % --- Data Extraction Logic ---
    if isfield(data, 't_plot')
        t = data.t_plot(:);
        
        % Handle variable names found in Triangle tracking data
        if isfield(data, 'val_plot')
            vals = data.val_plot;
        elseif isfield(data, 'val_plot_filtered')
            vals = data.val_plot_filtered;
        else
            vals = [];
        end
        
        if ~isempty(vals)
            phi_meas     = vals(:, 1);
            theta_meas   = vals(:, 2);
            phi_target   = vals(:, 3);
            theta_target = vals(:, 4);
        elseif isfield(data, 'phi_actual') 
            phi_meas     = data.phi_actual(:);
            theta_meas   = data.theta_actual(:);
            phi_target   = data.phi_target(:);
            theta_target = data.theta_target(:);
        else
            continue;
        end
    else
        continue;
    end
    
    % --- Metadata Parsing (Triangle specific: 50, 60, 70) ---
    tokens = regexp(filename_str, '(\d+\.\d+)Length', 'tokens');
    if ~isempty(tokens)
        rad_val = num2str(str2double(tokens{1}{1}) * 1000); 
    else
        rad_val = '??';
    end
    
    freq_label = '';
    if contains(filename_str, 'Time30'),     freq_label = '\pi/15';
    elseif contains(filename_str, 'Time60'), freq_label = '\pi/30';
    end
    
    name_str = sprintf('R_{a,%s} = %s mm', freq_label, rad_val);
    
    % Plot Phi
    figure(fig_phi); hold on; grid on; box on;
    plot(t, phi_meas, 'LineWidth', 1.2, 'Color', colors(k, :), 'DisplayName', name_str);
    plot(t, phi_target, '--', 'Color', [0.5 0.5 0.5, 0.4], 'HandleVisibility', 'off'); 
    
    % Plot Theta
    figure(fig_theta); hold on; grid on; box on;
    plot(t, theta_meas, 'LineWidth', 1.2, 'Color', colors(k, :), 'DisplayName', name_str);
    plot(t, theta_target, '--', 'Color', [0.5 0.5 0.5, 0.4], 'HandleVisibility', 'off'); 
end

%% Final Formatting and Export Loop
figs = [fig_phi, fig_theta];
y_labels = {'\phi (rad)', '\theta (rad)'};

for i = 1:length(figs)
    curr_fig = figs(i);
    figure(curr_fig);
    ax = gca;
    drawnow;
    
    xlabel('Time (s)', 'FontSize', 10);
    ylabel(y_labels{i}, 'FontSize', 10);
    set(ax, 'XLim', [0 120]); % Match experimental duration
    
    % --- Centered Top Legend (3 Columns, Compact) ---
    lgd = legend('show');
    if ~isempty(lgd)
        set(lgd, 'Interpreter', 'tex', ...
                 'Location', 'northoutside', ... 
                 'Orientation', 'horizontal', ... 
                 'NumColumns', 3, ...          
                 'IconColumnWidth', 5, ...     
                 'EdgeColor', 'k', ...         
                 'FontSize', 7);               
    end
    
    % --- Publication Export Setup (3.5x2.5) ---
    set(curr_fig, 'Units', 'inches', 'Position', [1, 1, 3.5, 2.5]); 
    set(ax, 'Units', 'normalized', 'Position', [0.18, 0.18, 0.75, 0.55]); % Manual axis fit
    
    set(curr_fig, 'PaperUnits', 'inches', 'PaperPosition', [0, 0, 3.5, 2.5], 'PaperSize', [3.5, 2.5]);
    set(ax, 'FontSize', 9, 'FontName', 'Helvetica');
end