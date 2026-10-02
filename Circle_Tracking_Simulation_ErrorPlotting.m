%% Unified Error Plotting Script - Error vs Time (50 Hz)
% Path to the folder containing your .mat error files
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\Circular_ExperimentData\Error_Grouping';

% 1. Identify files
file_list = dir(fullfile(target_path, '*.mat'));
colors = lines(length(file_list));

% 2. Initialize Figure 1: Bending Angle (Phi) Error
fig_phi = figure('Name', 'Bending Angle Error', 'Color', 'w', 'Position', [100 100 600 450]);
hold on; box on; grid on;

% 3. Initialize Figure 2: Rotational (Theta) Error
fig_theta = figure('Name', 'Rotational Error', 'Color', 'w', 'Position', [750 100 600 450]);
hold on; box on; grid on;

% 4. Loop through files
for k = 1:length(file_list)
    filename_str = file_list(k).name;
    data = load(fullfile(target_path, filename_str));
    
    % --- Updated Robust Variable Check ---
    % Checks for t_plot and then identifies if data is in val_plot1 or val_plot1_filtered
    if isfield(data, 't_plot')
        t = data.t_plot(:); 
        
        if isfield(data, 'val_plot1')
            val_data = data.val_plot1;
        elseif isfield(data, 'val_plot1_filtered')
            val_data = data.val_plot1_filtered;
        else
            % Skip if neither variable exists
            continue; 
        end
        
        % Data check: Ensure Nx2 orientation (Col 1: Phi error, Col 2: Theta error)
        if size(val_data, 1) == 2, val_data = val_data'; end
        phi_error   = val_data(:, 1); 
        theta_error = val_data(:, 2);
        
        % --- Parse filename for phi and Frequency ---
        phi_val = '';
        if contains(filename_str, '0.4Phi'),     phi_val = '0.4';
        elseif contains(filename_str, '0.6Phi'), phi_val = '0.6';
        elseif contains(filename_str, '0.8Phi'), phi_val = '0.8';
        end
        
        freq_label = '';
        if contains(filename_str, 'Time30'),     freq_label = '\pi/15';
        elseif contains(filename_str, 'Time60'), freq_label = '\pi/30';
        end
        
        % --- Legend Formatting: Separate names for each graph ---
        if ~isempty(phi_val) && ~isempty(freq_label)
            % Name for Phi Graph: phi_{a, freq} = val
            name_phi   = sprintf('\\phi_{a, %s} = %s', freq_label, phi_val);
            % Name for Theta Graph: theta_{a, freq} = val
            name_theta = sprintf('\\phi_{a, %s} = %s', freq_label, phi_val);
        else
            name_phi   = strrep(filename_str, '_', ' '); 
            name_theta = name_phi;
        end
        
        % 5. Plot vs Time in Figure 1 (Phi)
        figure(fig_phi);
        plot(t, phi_error, 'LineWidth', 1.5, 'Color', colors(k, :), ...
             'DisplayName', name_phi);
        
        % 6. Plot vs Time in Figure 2 (Theta)
        figure(fig_theta);
        plot(t, theta_error, 'LineWidth', 1.5, 'Color', colors(k, :), ...
             'DisplayName', name_theta);
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
    % Original: [0.15, 0.18, 0.75, 0.60]
    % New: [0.15, 0.12, 0.75, 0.55] 
    % (We lowered the bottom to 0.12 and reduced height to 0.55)
    set(ax, 'Units', 'normalized');
    set(ax, 'Position', [0.15, 0.17, 0.75, 0.58]); 
    
    set(curr_fig, 'PaperUnits', 'inches');
    set(curr_fig, 'PaperPosition', [0, 0, 3.5, 2.5]);
    set(curr_fig, 'PaperSize', [3.5, 2.5]);
    
    % Final font settings
    set(ax, 'FontSize', 9, 'FontName', 'Helvetica');
end