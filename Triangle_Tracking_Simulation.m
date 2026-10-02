%% ESP32-CONSISTENT psi-rate -> tendon-rate simulation TRIANGLE (TIP TOWARD X)
clear; clc; close all; 

% --- Pre-calculation of Triangle Trajectory ---
L_backbone = 0.256;             
t_steps = 1000;                 
t_traj = linspace(0, 1, t_steps); 
side_length = 0.09;             
offset = 0.001;                 

% Define Triangle Vertices
% v1: Tip pointing RIGHT (towards +X axis)
% v2: Bottom Left
% v3: Top Left
v1 = [side_length, 0];
v2 = [-side_length * sin(pi/6), -side_length * cos(pi/6)];
v3 = [-side_length * sin(pi/6),  side_length * cos(pi/6)];

x_des_raw = zeros(size(t_traj));
y_des_raw = zeros(size(t_traj));

for i = 1:t_steps
    tp = t_traj(i);
    if tp <= 1/3 % Side 1: v1 to v3 (CCW)
        s = tp * 3;
        x_des_raw(i) = v1(1) + s * (v3(1) - v1(1));
        y_des_raw(i) = v1(2) + s * (v3(2) - v1(2));
    elseif tp <= 2/3 % Side 2: v3 to v2 (CCW)
        s = (tp - 1/3) * 3;
        x_des_raw(i) = v3(1) + s * (v2(1) - v3(1));
        y_des_raw(i) = v3(2) + s * (v2(2) - v3(2));
    else % Side 3: v2 to v1 (CCW)
        s = (tp - 2/3) * 3;
        x_des_raw(i) = v2(1) + s * (v1(1) - v2(1));
        y_des_raw(i) = v2(2) + s * (v1(2) - v2(2));
    end
end

% Apply offset
x_des_raw = x_des_raw + offset;
y_des_raw = y_des_raw + offset;

% --- Configuration Space Mapping (IK) ---
phi_traj = zeros(size(t_traj));   
theta_traj_raw = atan2(y_des_raw, x_des_raw); 

% 1. UNWRAP THETA BEFORE INTERPOLATION
theta_traj_unwrapped = unwrap(theta_traj_raw);

for i = 1:length(t_traj)
    R = sqrt(x_des_raw(i)^2 + y_des_raw(i)^2); 
    phi_fun = @(p) L_backbone*(1-cos(p))/p - R; 
    phi_traj(i) = fzero(phi_fun, 0.1); 
    if phi_traj(i) < 1e-4, phi_traj(i) = 1e-4; end 
end

%% ---------------- Parameters (MATCH ESP32) ----------------
r_disk = 0.0304;    
dt = 0.02;          
T  = 30;            
t  = 0:dt:T;        
N  = length(t);     

phi_des   = interp1(linspace(0, T, length(phi_traj)), phi_traj, t, 'linear', 'extrap');

% 2. INTERPOLATE THE UNWRAPPED TRAJECTORY
theta_des_unwrapped = interp1(linspace(0, T, length(theta_traj_unwrapped)), theta_traj_unwrapped, t, 'linear', 'extrap');

% 3. WRAP BACK TO [-pi, pi] (if sending wrapped angles to ESP32)
theta_des = wrapToPi(theta_des_unwrapped);

%% ---------------- Initialize states -----------------------
phi   = phi_des(1);     
theta = theta_des(1);   
phi_prev   = phi;       
theta_prev = theta;     
l = zeros(3,1);         

%% ---------------- Main simulation loop --------------------
l_hist   = zeros(3,N);
psi_hist = zeros(2,N);

for k = 1:N 
    if k > 1
        dPhi = phi_des(k) - phi_prev;
        dTheta = theta_des(k) - theta_prev;
        dTheta = atan2(sin(dTheta), cos(dTheta));
        
        phi_dot   = dPhi / dt;
        theta_dot = dTheta / dt;
        phi_safe = max(abs(phi), 1e-4) * sign(phi); 
        
        J = zeros(3,2);
        J(1,:) = [-r_disk*cos(theta),                     phi_safe*r_disk*sin(theta)];
        J(2,:) = [-r_disk*(sqrt(3)*sin(theta)-cos(theta))/2, ...
                  -phi_safe*r_disk*(sqrt(3)*cos(theta)+sin(theta))/2];
        J(3,:) = [ r_disk*(sqrt(3)*sin(theta)+cos(theta))/2, ...
                   phi_safe*r_disk*(sqrt(3)*cos(theta)-sin(theta))/2];
        
        dL = J * [phi_dot; theta_dot];
        l = l + dL * dt;
        phi   = phi   + phi_dot   * dt;
        theta = theta + theta_dot * dt;
    end
    
    l_hist(:,k)   = l;
    psi_hist(:,k) = [phi; theta];
    phi_prev      = phi;
    theta_prev    = theta;
end

%% ---------------- Visualization ---------------------------
figure(1); clf; hold on; grid on; axis equal;
view(0,90); 
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
nx = 15; 
s = linspace(0, 1, nx);
for k = 1:10:N 
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
title('Triangle Trajectory (Tip Pointing Toward X)');