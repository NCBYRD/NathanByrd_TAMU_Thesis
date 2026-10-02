%% Unified Multi-File Trajectory Plotting Script - Updated Legend
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\Circular_ExperimentData\NDI_Grouping';

% 1. Constants
L_backbone = 256; 
phi_targets = [0.4, 0.6, 0.8];
theta_samples = linspace(0, 2*pi, 200); 

% 2. Identify files
file_list = dir(fullfile(target_path, '*.mat'));

% 3. Initialize Figure
fig_overlay = figure('Name', 'NDI Overlay vs Desired Trajectories', 'Color', 'w');
hold on; grid on;
colors = lines(length(file_list));

% 4. Plot Desired Reference Circles (phi_d)
for p = 1:length(phi_targets)
    phi = phi_targets(p);
    R_ref = L_backbone * (1 - cos(phi)) / phi;
    x_ref = R_ref * cos(theta_samples);
    y_ref = R_ref * sin(theta_samples);
    
    plot(x_ref, y_ref, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, ...
        'DisplayName', sprintf('\\phi_{d} = %.1f', phi));
end

% 5. Loop through files and overlay data (phi_a)
for k = 1:length(file_list)
    filename_str = file_list(k).name;
    data = load(fullfile(target_path, filename_str));
    
    if isfield(data, 'x_actual')
        x_p = data.x_actual; y_p = data.y_actual;
    elseif isfield(data, 'x_final')
        x_p = data.x_final;  y_p = data.y_final;
    else
        continue; 
    end
    
    % --- Parse filename for phi_a and Frequency ---
    phi_val = '';
    if contains(filename_str, '0.4Phi'), phi_val = '0.4';
    elseif contains(filename_str, '0.6Phi'), phi_val = '0.6';
    elseif contains(filename_str, '0.8Phi'), phi_val = '0.8';
    end
    
    freq_val = '';
    if contains(filename_str, 'Time30'), freq_val = '\pi/15';
    elseif contains(filename_str, 'Time60'), freq_val = '\pi/30';
    end
    
    % Build the streamlined name: phi_a = 0.6, Frequency 2pi/30
    if ~isempty(phi_val) && ~isempty(freq_val)
        streamlined_name = sprintf('\\phi_{a, %s} = %s', freq_val, phi_val);
    else
        streamlined_name = strrep(filename_str, '_', ' '); 
    end
    
    plot(x_p, y_p, 'LineWidth', 1.5, 'Color', colors(k, :), ...
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