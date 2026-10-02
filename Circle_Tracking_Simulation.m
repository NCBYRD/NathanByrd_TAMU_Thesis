%% ESP32-CONSISTENT psi-rate -> tendon-rate simulation CIRCLE (FINAL WRAPPED)
clear; clc; close all; 

% --- Pre-calculation of Circular Trajectory ---
L_backbone = 0.256;             % Length of the robot backbone in meters
t_traj = linspace(0, 2*pi, 1000); % Parametric vector for one full rotation

phi_const = 0.6;                % Bending magnitude
phi_traj = phi_const * ones(size(t_traj)); 
theta_traj = t_traj;            % 0 to 2*pi sweep

%% ---------------- Parameters (MATCH ESP32) ----------------
r_disk = 0.0304;    % distance from backbone to tendon (m)
dt = 0.02;          % 50 Hz controller update rate
T  = 60;            % simulation duration (s)
t  = 0:dt:T;        % time vector
N  = length(t);     % total steps

% Interpolate trajectory
phi_des   = interp1(linspace(0, T, length(phi_traj)), phi_traj, t, 'linear', 'extrap');
theta_des_raw = interp1(linspace(0, T, length(theta_traj)), theta_traj, t, 'linear', 'extrap');

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
        % This keeps the internal state consistently between -3.14 and 3.14
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
subplot(2,1,1);
plot(t, psi_hist(2,:), 'LineWidth', 1.5);
hold on;
yline(pi, 'r--'); yline(-pi, 'r--');
grid on;
ylabel('Theta (rad)');
title('Theta Angle Wrapped between -3.14 and 3.14');
ylim([-4 4]);

subplot(2,1,2);
plot(t, l_hist);
grid on;
ylabel('Tendon Length (m)');
xlabel('Time (s)');
title('Tendon Displacements');

% 3D Plot
figure(2); clf; hold on; grid on; axis equal;
view(45,30);
target_x = (L_backbone*(1-cos(phi_const))/phi_const) * cos(t_traj);
target_y = (L_backbone*(1-cos(phi_const))/phi_const) * sin(t_traj);
target_z = (L_backbone*sin(phi_const)/phi_const) * ones(size(t_traj));
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