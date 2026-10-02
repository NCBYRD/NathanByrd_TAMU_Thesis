%% --- 1. Parameters & Tiled Circular Reference (mm) ---
L_backbone = 256;        
dt = 0.02;               
T_ref = 30;              % 60s per rotation
T_total = 180;           
t = 0:dt:T_total;        

% Tiling logic (Matches your square script)
t_wrapped = mod(t, T_ref);

% Calculate Circular Radius
R_arc = (L_backbone * (1 - cos(0.4)) / 0.4); 

% Generate Desired Circular Path (Continuous, Tiled)
theta_des = (2 * pi / T_ref) * t_wrapped; 
x_des = R_arc * cos(theta_des);
y_des = R_arc * sin(theta_des);

%% --- 2. Process Experimental Data (mm) ---
% Assuming flat_data is 6xN from your NDI simulation
t_exp_raw = linspace(0, T_total, size(flat_data, 2));
x_actual_raw = flat_data(4, :); 
y_actual_raw = -flat_data(5, :);

% Interpolate to match time 't'
x_interp_raw = interp1(t_exp_raw, x_actual_raw, t, 'linear', 'extrap');
y_interp_raw = interp1(t_exp_raw, y_actual_raw, t, 'linear', 'extrap');

% --- ROTATION CORRECTION (To match Square logic) ---
theta_deg = -2; % Adjust this to align your NDI with Robot Frame
theta_rad = deg2rad(theta_deg);
R = [cos(theta_rad), -sin(theta_rad); sin(theta_rad), cos(theta_rad)];

rotated_coords = R * [x_interp_raw; y_interp_raw];
x_interp = rotated_coords(1, :);
y_interp = rotated_coords(2, :);

%% --- 3. ROBUST CENTERING & SYNC ---
steady_idx = (t > 40) & (t < 160); % 120s window (2 full laps)

% 1. Geometric Centering 
actual_center_x = mean(x_interp(steady_idx));
actual_center_y = mean(y_interp(steady_idx));

% Shift NDI data to the Desired Center
x_centered = x_interp - actual_center_x + mean(x_des(steady_idx));
y_centered = y_interp - actual_center_y + mean(y_des(steady_idx));

% 2. Sync Phase (Temporal Alignment via Cross-Correlation)
[corr, lags] = xcorr(x_des(steady_idx), x_centered(steady_idx));
[~, max_idx] = max(corr);
sample_lag = lags(max_idx);

% 3. Apply the shift (The circshift you prefer)
x_actual = circshift(x_centered, sample_lag);
y_actual = circshift(y_centered, sample_lag);

%% --- 4. CALCULATE GEOMETRIC VS. TRACKING ERROR ---

% 1. Tracking Error (Time-Synced - includes the lag you see in the plots)
error_euclidean = sqrt((x_des - x_actual).^2 + (y_des - y_actual).^2);
rmse_tracking = sqrt(mean(error_euclidean(steady_idx).^2));

% 2. Geometric Error (Radial - ignores the lag)
% We calculate how far each point is from the center (0,0)
actual_radius = sqrt(x_centered.^2 + y_centered.^2); 
error_radial = actual_radius - R_arc; 

% Geometric RMSE
rmse_geometric = sqrt(mean(error_radial(steady_idx).^2));

% 3. Print Comparison
fprintf('\n--- Performance Metrics ---\n');
fprintf('Tracking RMSE (with lag): %.3f mm\n', rmse_tracking);
fprintf('Geometric RMSE (true path): %.3f mm\n', rmse_geometric);
fprintf('Max Radial Deviation:      %.3f mm\n', max(abs(error_radial(steady_idx))));

%% --- 5. Visualization  ---
t_steady = t(steady_idx);
t_rel = t_steady - t_steady(1); 

fig1 = figure(1); clf; set(gcf, 'Color', 'w');
subplot(1,2,1);
plot(x_des(steady_idx), y_des(steady_idx), 'r--', 'LineWidth', 2); hold on;
plot(x_actual(steady_idx), y_actual(steady_idx), 'b');
axis equal; grid on;
legend('Desired Circle', 'Actual (Synced)');
title('Circular Geometry (XY Plane)');

subplot(1,2,2);
plot(t_rel, error_radial(steady_idx), 'Color', [0.2 0.7 0.2]);
grid on; xlabel('Time (s)'); ylabel('Euclidean Error (mm)');
title(['Steady-State RMSE: ', num2str(rmse_geometric, '%.3f'), ' mm']);

% Figure 2: Temporal Comparison
fig2 = figure(2); clf; set(gcf, 'Color', 'w');
subplot(2,1,1);
% Use t_rel for X-axis tracking
plot(t_rel, x_des(steady_idx), 'r--', t_rel, x_actual(steady_idx), 'b');
grid on; ylabel('X (mm)'); 
title('X-Axis Tracking');

subplot(2,1,2);
% Use t_rel for Y-axis tracking
plot(t_rel, y_des(steady_idx), 'r--', t_rel, y_actual(steady_idx), 'b');
grid on; ylabel('Y (mm)'); xlabel('Time (s)'); 
title('Y-Axis Tracking');

%% ---------------- 6. EXPORT & SAVE SECTION ----------------
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\TheAviFolder';
if ~exist(target_path, 'dir'), mkdir(target_path); end

timestamp = datestr(now, 'yyyy-mm-dd_HHMM');
base_filename = ['Circular_NDI_Analysis_mm_', timestamp];

exportgraphics(fig1, fullfile(target_path, [base_filename, '_Spatial.png']), 'Resolution', 300);
exportgraphics(fig2, fullfile(target_path, [base_filename, '_Temporal.png']), 'Resolution', 300);

save(fullfile(target_path, [base_filename, '_Data.mat']), ...
    't', 'x_des', 'y_des', 'x_actual', 'y_actual', 'rmse_val', 'time_lag', ...
    'actual_center_x', 'actual_center_y', 'error_euclidean');

fileID = fopen(fullfile(target_path, [base_filename, '_Report.txt']), 'w');
fprintf(fileID, '--- NDI Experiment Tracking Report (mm) ---\n');
fprintf(fileID, 'RMSE (Steady-State): %.3f mm\n', rmse_val);
fprintf(fileID, 'Measured Center Offset (X,Y): [%.2f, %.2f] mm\n', actual_center_x, actual_center_y);
fprintf(fileID, 'Detected Temporal Lag: %.3f seconds\n', time_lag);
fclose(fileID);

fprintf('Done! Files saved in mm to: %s\n', target_path);
Calculate the measured diameter from your circle fit
measured_diameter = 2 * sqrt(coeffs(1)^2/4 + coeffs(2)^2/4 - coeffs(3));
fprintf('Desired Diameter: %.2f mm\n', 2 * R_arc);
fprintf('Measured Diameter: %.2f mm\n', measured_diameter);
