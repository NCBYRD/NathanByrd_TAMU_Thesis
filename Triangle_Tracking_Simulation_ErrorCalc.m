%% ESP32-CONSISTENT: Triangle Trajectory (180s Total, 60s Tiled Reference)
% Units: mm | Experiment: 180s | Ref: 60s loop
target_path = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\TheAviFolder';

%% ---------------- 1. Parameters & Tiled Reference (mm) ----------------
L_backbone = 256;        
dt = 0.02;               
T_ref = 30;              
T_total = 180;           
t = 0:dt:T_total;        

% Tiling logic
t_wrapped = mod(t, T_ref);
t_norm = t_wrapped / T_ref; 

% Triangle Parameters in mm
side_mm = 70.0; % Distance from center to vertices
offset_mm = 1.0;

% Define Triangle Vertices (Equilateral, Tip pointing UP toward +Y)
v1 = [0, side_mm];                                      % Top Tip
v2 = [-side_mm * cos(pi/6), -side_mm * sin(pi/6)];      % Bottom Left
v3 = [side_mm * cos(pi/6), -side_mm * sin(pi/6)];       % Bottom Right

% Generate Desired Triangle Path
x_des = zeros(size(t));
y_des = zeros(size(t));

for i = 1:length(t)
    tp = t_norm(i);
    if tp <= 1/3 % Side 1: v1 to v2
        s = tp * 3;
        x_des(i) = v1(1) + s * (v2(1) - v1(1));
        y_des(i) = v1(2) + s * (v2(2) - v1(2));
    elseif tp <= 2/3 % Side 2: v2 to v3
        s = (tp - 1/3) * 3;
        x_des(i) = v2(1) + s * (v3(1) - v2(1));
        y_des(i) = v2(2) + s * (v3(2) - v2(2));
    else % Side 3: v3 to v1
        s = (tp - 2/3) * 3;
        x_des(i) = v3(1) + s * (v1(1) - v3(1));
        y_des(i) = v3(2) + s * (v1(2) - v3(2));
    end
end

x_des = x_des + offset_mm;
y_des = y_des + offset_mm;

%% ---------------- 2. Process Experimental Data (mm) ----------------
% (Assumes flat_data is already loaded in workspace from your previous steps)
t_exp_raw = linspace(0, T_total, size(flat_data, 2));
x_actual_raw = (flat_data(4, :)); 
y_actual_raw = (flat_data(5, :));

% Interpolate to match time 't'
x_interp_raw = interp1(t_exp_raw, x_actual_raw, t, 'linear', 'extrap');
y_interp_raw = interp1(t_exp_raw, y_actual_raw, t, 'linear', 'extrap');

% --- ROTATION CORRECTION ---
theta_deg = 2; 
theta_rad = deg2rad(theta_deg);
R_mat = [cos(theta_rad), -sin(theta_rad); 
         sin(theta_rad),  cos(theta_rad)];
rotated_coords = R_mat * [x_interp_raw; y_interp_raw];
x_interp = rotated_coords(1, :);
y_interp = rotated_coords(2, :);

%% ---------------- 3. ROBUST CENTERING & SYNC ----------------
steady_idx = (t > 40) & (t < 160);

% 1. Geometric Centering
actual_center_x = mean(x_interp(steady_idx));
actual_center_y = mean(y_interp(steady_idx));
x_centered = x_interp - actual_center_x + mean(x_des(steady_idx));
y_centered = y_interp - actual_center_y + mean(y_des(steady_idx));

% 2. Sync Phase (Temporal Alignment)
[corr, lags] = xcorr(x_des(steady_idx), x_centered(steady_idx));
[~, max_idx] = max(corr);
sample_lag = lags(max_idx);

% 3. Apply the shift
x_actual = circshift(x_centered, sample_lag);
y_actual = circshift(y_centered, sample_lag);

%% ---------------- 4. Calculate Error (mm) ----------------
error_x = x_des - x_actual;
error_y = y_des - y_actual;
error_euclidean = sqrt(error_x.^2 + error_y.^2);
rmse_val = sqrt(mean(error_euclidean(steady_idx).^2));
time_lag = sample_lag * dt;

%% ---------------- 5. Visualization ----------------
t_steady = t(steady_idx);
t_rel = t_steady - t_steady(1); 

fig1 = figure(1); clf; set(gcf, 'Color', 'w');
subplot(1,2,1);
plot(x_des(steady_idx), y_des(steady_idx), 'r--', 'LineWidth', 2); hold on;
plot(x_actual(steady_idx), y_actual(steady_idx), 'b');
axis equal; grid on;
legend('Desired', 'Actual (Synced)');
title('Triangle Geometry (XY Plane)');

subplot(1,2,2);
plot(t_rel, error_euclidean(steady_idx), 'Color', [0.2 0.7 0.2]);
grid on; xlabel('Time (s)'); ylabel('Euclidean Error (mm)');
title(['Steady-State RMSE: ', num2str(rmse_val, '%.3f'), ' mm']);

fig2 = figure(2); clf; set(gcf, 'Color', 'w');
subplot(2,1,1);
plot(t_rel, x_des(steady_idx), 'r--', t_rel, x_actual(steady_idx), 'b');
grid on; ylabel('X (mm)'); title('X-Axis Tracking');
subplot(2,1,2);
plot(t_rel, y_des(steady_idx), 'r--', t_rel, y_actual(steady_idx), 'b');
grid on; ylabel('Y (mm)'); xlabel('Time (s)'); title('Y-Axis Tracking');

%% ---------------- 6. EXPORT & SAVE ----------------
if ~exist(target_path, 'dir'), mkdir(target_path); end
timestamp = datestr(now, 'yyyy-mm-dd_HHMM');
base_filename = ['Triangle_NDI_Analysis_mm_', timestamp];

exportgraphics(fig1, fullfile(target_path, [base_filename, '_Spatial.png']), 'Resolution', 300);
exportgraphics(fig2, fullfile(target_path, [base_filename, '_Temporal.png']), 'Resolution', 300);

save(fullfile(target_path, [base_filename, '_Data.mat']), 't', 'x_des', 'y_des', 'x_actual', 'y_actual', 'rmse_val');

fileID = fopen(fullfile(target_path, [base_filename, '_Report.txt']), 'w');
fprintf(fileID, 'Steady-State RMSE: %.3f mm\n', rmse_val);
fprintf(fileID, 'Detected Lag: %.3f s\n', time_lag);
fclose(fileID);

fprintf('Done! Steady-State RMSE: %.3f mm\n', rmse_val);