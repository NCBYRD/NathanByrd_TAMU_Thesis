%% Unified Task-Space Script: Phi and Theta vs Time
% Update path to your TaskSpace_Grouping folder
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStructure_LoadBearingTests\LoadBearing\LoadBearingTest_ZeroWeight';

file_list = dir(fullfile(target_path, '*.mat'));
if isempty(file_list), error('No files found in path.'); end
colors = lines(length(file_list));

% Initialize Figures
fig_phi = figure('Name', 'Phi Tracking', 'Color', 'w', 'Position', [100 100 600 450]); hold on; grid on;
fig_theta = figure('Name', 'Theta Tracking', 'Color', 'w', 'Position', [750 100 600 450]); hold on; grid on;

for k = 1:length(file_list)
    filename_str = file_list(k).name;
    data = load(fullfile(target_path, filename_str));
    
    if isfield(data, 't_plot')
        t = data.t_plot(:);
        % Handle matrix format found in provided .mat (val_plot: 6001x4)
        if isfield(data, 'val_plot')
            phi_meas = data.val_plot(:, 1);
            theta_meas = data.val_plot(:, 2);
            phi_target = data.val_plot(:, 3);
            theta_target = data.val_plot(:, 4);
        % Handle individual variable format from your screenshot
        elseif isfield(data, 'phi_actual') 
            phi_meas = data.phi_actual(:);
            theta_meas = data.theta_actual(:);
            phi_target = data.phi_target(:);
            theta_target = data.theta_target(:);
        else
            continue;
        end
    else
        continue;
    end
    
    % --- Metadata Parsing ---
    phi_val = '';
    if contains(filename_str, '0.4Phi'), phi_val = '0.4';
    elseif contains(filename_str, '0.6Phi'), phi_val = '0.6';
    elseif contains(filename_str, '0.8Phi'), phi_val = '0.8';
    end
    
    freq_label = '';
    if contains(filename_str, 'Time30'), freq_label = '\pi/15';
    elseif contains(filename_str, 'Time60'), freq_label = '\pi/30';
    end
    
    % --- Legends ---
    name_phi = sprintf('\\phi_{a, %s} = %s', freq_label, phi_val);
    name_theta = sprintf('\\theta_{a, %s} = %s', freq_label, phi_val);

    % Plot Phi
    figure(fig_phi);
    plot(t, phi_meas, 'LineWidth', 1.5, 'Color', colors(k, :), 'DisplayName', name_phi);
    plot(t, phi_target, '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off'); % Target dashed

    % Plot Theta
    figure(fig_theta);
    plot(t, theta_meas, 'LineWidth', 1.5, 'Color', colors(k, :), 'DisplayName', name_theta);
    plot(t, theta_target, '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off'); % Target dashed
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