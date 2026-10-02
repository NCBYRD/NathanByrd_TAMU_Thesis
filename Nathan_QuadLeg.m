%% ESP32-CONSISTENT psi-rate -> tendon-rate simulation FROM DATA (FINAL WRAPPED)
clear; clc; close all;

%% ---------------- Load trajectory data ---------------------
% Expects Natha_quad_leg_1.mat with:
%   leg1_config : 2 x N   -> row 1 = theta (rad), row 2 = phi (rad)
%   t           : N x 1   -> timestamps (s), non-uniform spacing
%
% NOTE: row assignment (theta vs phi) was inferred from data ranges
% (row 1 stays small/near zero -> theta; row 2 ramps 0 -> ~1.7 rad -> phi).
% Flip theta_data/phi_data below if your convention is reversed.

data = load('Natha_quad_leg_1.mat');

t_data     = data.t(:).';           % force row vector
theta_data = data.leg1_config(1,:); % bending direction
phi_data   = data.leg1_config(2,:); % bending magnitude

PHI_SCALE = 0.87;   % scale factor applied to bending magnitude
phi_data  = phi_data * PHI_SCALE;

L_backbone = 0.256;                 % Length of the robot backbone in meters

%% ---------------- Parameters (MATCH ESP32) ----------------
r_disk = 0.0304;    % distance from backbone to tendon (m)
dt = 0.02;          % 50 Hz controller update rate

START_IDX    = 185; % start the resampled (50 Hz) trajectory at this index
TIME_STRETCH = 3;   % slow the motion down by this factor (3 = 3x slower); tweak as needed

% --- Pass 1: resample the raw data onto the native 50 Hz grid ---
T0 = t_data(end);
t0 = 0:dt:T0;                        % this is the 1x2001 double grid
phi_des0       = interp1(t_data, phi_data,   t0, 'linear', 'extrap');
theta_des0_raw = interp1(t_data, theta_data, t0, 'linear', 'extrap');
theta_des0     = atan2(sin(theta_des0_raw), cos(theta_des0_raw));

% --- Trim to start at START_IDX, re-zero time ---
phi_trim   = phi_des0(START_IDX:end);
theta_trim = theta_des0(START_IDX:end);
t_trim     = t0(START_IDX:end) - t0(START_IDX);

% --- Stretch time, then re-interpolate onto a fresh fixed 50 Hz grid ---
t_stretched = t_trim * TIME_STRETCH;
T  = t_stretched(end);   % simulation duration (s), after trim + stretch
t  = 0:dt:T;             % final time vector
N  = length(t);          % total steps

phi_des       = interp1(t_stretched, phi_trim,   t, 'linear', 'extrap');
theta_des_raw = interp1(t_stretched, theta_trim, t, 'linear', 'extrap');

% Force the desired trajectory to be between -pi and pi
theta_des = atan2(sin(theta_des_raw), cos(theta_des_raw));

%% ---------------- Initialize states -----------------------
phi   = phi_des(1);
theta = theta_des(1);   % Start within [-3.14, 3.14]
phi_prev   = phi;
theta_prev = theta;
l = zeros(3,1);         % Tendon displacement (m)

%% ---------------- Storage ---------------------------------
l_hist   = zeros(3,N);
psi_hist = zeros(2,N);

%% ---------------- Main simulation loop --------------------
for k = 1:N
    if k > 1
        % 1. Calculate Difference
        dPhi = phi_des(k) - phi_prev;
        dTheta = theta_des(k) - theta_prev;

        % 2. Short-path wrapping (prevents jumps when crossing +/- pi)
        dTheta = atan2(sin(dTheta), cos(dTheta));

        phi_dot   = dPhi / dt;
        theta_dot = dTheta / dt;

        % 3. JACOBIAN: Mapping configuration rates to tendon rates
        phi_safe = max(abs(phi), 1e-4) * sign(phi);
        if phi_safe == 0
            phi_safe = 1e-4; % guard against phi starting exactly at 0
        end

        J = zeros(3,2);
        J(1,:) = [-r_disk*cos(theta),                     phi_safe*r_disk*sin(theta)];
        J(2,:) = [-r_disk*(sqrt(3)*sin(theta)-cos(theta))/2, ...
                  -phi_safe*r_disk*(sqrt(3)*cos(theta)+sin(theta))/2];
        J(3,:) = [ r_disk*(sqrt(3)*sin(theta)+cos(theta))/2, ...
                   phi_safe*r_disk*(sqrt(3)*cos(theta)-sin(theta))/2];

        % 4. Update Tendon Lengths (Integration)
        dL = J * [phi_dot; theta_dot];
        l = l + dL * dt;

        % 5. Update Configuration (Simulated state)
        phi   = phi   + phi_dot   * dt;
        theta = theta + theta_dot * dt;

        % 6. FORCE STATE WRAPPING [-pi, pi]
        theta = atan2(sin(theta), cos(theta));
    end

    % Store
    l_hist(:,k)   = l;
    psi_hist(:,k) = [phi; theta];
    phi_prev      = phi;
    theta_prev    = theta;
end

%% ---------------- Visualization ---------------------------
figure(1); clf;
subplot(3,1,1);
plot(t, psi_hist(1,:), 'LineWidth', 1.5);
grid on;
ylabel('Phi (rad)');
title('Phi (Bending Magnitude)');

subplot(3,1,2);
plot(t, psi_hist(2,:), 'LineWidth', 1.5);
hold on;
yline(pi, 'r--'); yline(-pi, 'r--');
grid on;
ylabel('Theta (rad)');
title('Theta Angle Wrapped between -3.14 and 3.14');
ylim([-4 4]);

subplot(3,1,3);
plot(t, l_hist);
grid on;
ylabel('Tendon Length (m)');
xlabel('Time (s)');
title('Tendon Displacements');

% 3D Plot
figure(2); clf; hold on; grid on; axis equal;
view(45,30);

% Analytic tip position over time from the actual (phi_des, theta_des) data
target_x = zeros(1,N);
target_y = zeros(1,N);
target_z = zeros(1,N);
for k = 1:N
    pk = phi_des(k);
    tk = theta_des(k);
    if abs(pk) < 1e-4
        target_x(k) = 0;
        target_y(k) = 0;
        target_z(k) = L_backbone;
    else
        rho = L_backbone / pk;
        target_x(k) = rho * (1 - cos(pk)) * cos(tk);
        target_y(k) = rho * (1 - cos(pk)) * sin(tk);
        target_z(k) = rho * sin(pk);
    end
end
plot3(target_x, target_y, target_z, 'r--', 'LineWidth', 1);

nx = 15; s = linspace(0, 1, nx);
for k = 1:20:N
    pk = psi_hist(1,k);
    tk = psi_hist(2,k);
    pos = zeros(3, nx);
    for i = 1:nx
        si = s(i) * L_backbone;
        if abs(pk) < 1e-4
            pos(:,i) = [0; 0; si];
        else
            rho = L_backbone / pk;
            pos(:,i) = [rho * (1 - cos(si/rho)) * cos(tk);
                        rho * (1 - cos(si/rho)) * sin(tk);
                        rho * sin(si/rho)];
        end
    end
    plot3(pos(1,:), pos(2,:), pos(3,:), 'b-', 'LineWidth', 0.5);
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Backbone Shapes Along Leg 1 Trajectory');