%% ISS Lyapunov Function Analysis - Combined 2x3 layout
% Row 1: phi channel (|e|, V, Vdot)
% Row 2: theta channel (|e|, V, Vdot)

clear; close all; clc;

%% --- Load data ---
load('20260310_Square_TaskSpaceError_0.02Length_TotalTime60.MAT');

t       = t_plot(:);
e_phi   = val_plot1_filtered(:,1);
e_theta = val_plot1_filtered(:,2);
N  = length(t);
dt = mean(diff(t));

%% --- PID gains ---
K_P_phi = 150000;  K_I_phi = 0;  K_D_phi = 10000;
K_P_th  = 124500;  K_I_th  = 0;  K_D_th  = 20000;

alpha = 0.3;
R = 1.0;

%% --- Lyapunov computation function ---
function [e_abs, V, Vdot_analytic, Vdot_numeric, z, M, N_mat] = ...
        compute_lyapunov(e, P, Q, D, R, alpha, dt)
    N  = length(e);
    Omega   = 1 + alpha * D;
    B       = alpha * R / Omega * Q;
    M       = alpha * R / Omega * P;
    N_mat   = R / Omega;

    z = zeros(N,1);
    for k = 2:N
        z(k) = z(k-1) + 0.5*(e(k) + e(k-1))*dt;
    end

    e_abs = abs(e);
    V     = 0.5*R*e.^2 + 0.5*B*z.^2;
    Vdot_numeric  = gradient(V, dt);
    Vdot_analytic = -M * e.^2;
end

[e_phi_abs,   V_phi,   Vdot_phi_a,   Vdot_phi_n,   ~, M_phi,   ~] = ...
    compute_lyapunov(e_phi,   K_P_phi, K_I_phi, K_D_phi, R, alpha, dt);
[e_theta_abs, V_theta, Vdot_theta_a, Vdot_theta_n, ~, M_theta, ~] = ...
    compute_lyapunov(e_theta, K_P_th,  K_I_th,  K_D_th,  R, alpha, dt);

%% --- Combined 2x3 Plot ---
targetDir = 'C:\Users\ncbyrd\OneDrive - Texas A&M University\Research\PHD Students\Nathan\LatticeStructure\LatticeStruc_Matlab\taskspacecode\NDI Magnetic Tracking\Experiment_TrackingData\LyapunovFigures';

figure('Color','w','Position',[100 100 1300 550]);

dot_phi   = 'k';
dot_theta = [0.85 0.33 0.10];
ms = 4;
xl = [0 max(t)];

% --- Row 1: PHI ---
subplot(2,3,1);
plot(t, e_phi_abs, '.', 'Color', dot_phi, 'MarkerSize', ms);
ylabel('|e_\phi| (rad)');
xlabel('Time (s)');
title('[A] Bending angle error magnitude');
grid on; box on; xlim(xl);

subplot(2,3,2);
plot(t, V_phi, '.', 'Color', dot_phi, 'MarkerSize', ms);
ylabel('V_\phi');
xlabel('Time (s)');
title('[B] Lyapunov function value (\phi)');
grid on; box on; xlim(xl);

subplot(2,3,3);
plot(t, Vdot_phi_n, '.', 'Color', dot_phi, 'MarkerSize', ms, ...
    'DisplayName', '$\dot{V}_\phi$ numeric');
ylabel('$\dot{V}_\phi$', 'Interpreter', 'latex');
xlabel('Time (s)');
title('[C] Derivative of V_\phi');
grid on; box on; xlim(xl);

% --- Row 2: THETA ---
subplot(2,3,4);
plot(t, e_theta_abs, '.', 'Color', dot_theta, 'MarkerSize', ms);
ylabel('|e_\theta| (rad)');
xlabel('Time (s)');
title('[D] Orientation error magnitude');
grid on; box on; xlim(xl);

subplot(2,3,5);
plot(t, V_theta, '.', 'Color', dot_theta, 'MarkerSize', ms);
ylabel('V_\theta');
xlabel('Time (s)');
title('[E] Lyapunov function value (\theta)');
grid on; box on; xlim(xl);

subplot(2,3,6);
plot(t, Vdot_theta_n, '.', 'Color', dot_theta, 'MarkerSize', ms, ...
    'DisplayName', '$\dot{V}_\theta$ numeric');
ylabel('$\dot{V}_\theta$', 'Interpreter', 'latex');
xlabel('Time (s)');
title('[F] Derivative of V_\theta');
grid on; box on; xlim(xl);

sgtitle('Lyapunov Analysis: Bending (\phi) and Orientation (\theta) Channels');

%% --- Export ---
filePath = fullfile(targetDir, 'Lyapunov_combined.png');
exportgraphics(gcf, filePath, 'Resolution', 300);

%% --- Summary statistics ---
fprintf('--- Phi channel ---\n');
fprintf('Omega_phi = %.4f,  B_phi = %.4f,  M_phi = %.4f\n', ...
    1+alpha*K_D_phi, alpha*R/(1+alpha*K_D_phi)*K_I_phi, M_phi);
fprintf('Mean |e_phi|   = %.4f rad\n', mean(e_phi_abs));
fprintf('Max  |e_phi|   = %.4f rad\n', max(e_phi_abs));
fprintf('Mean V_phi     = %.4f\n', mean(V_phi));
fprintf('Max  V_phi     = %.4f\n', max(V_phi));
fprintf('Fraction Vdot_phi (numeric) <= 0: %.1f%%\n\n', ...
    100*sum(Vdot_phi_n<=0)/N);

fprintf('--- Theta channel ---\n');
fprintf('Omega_theta = %.4f,  B_theta = %.4f,  M_theta = %.4f\n', ...
    1+alpha*K_D_th, alpha*R/(1+alpha*K_D_th)*K_I_th, M_theta);
fprintf('Mean |e_theta| = %.4f rad\n', mean(e_theta_abs));
fprintf('Max  |e_theta| = %.4f rad\n', max(e_theta_abs));
fprintf('Mean V_theta   = %.4f\n', mean(V_theta));
fprintf('Max  V_theta   = %.4f\n', max(V_theta));
fprintf('Fraction Vdot_theta (numeric) <= 0: %.1f%%\n', ...
    100*sum(Vdot_theta_n<=0)/N);