%% BALL AND PLATE SYSTEM - LQR DESIGN & STABILITY PROOF
% ============================================================
% Course:  Control Systems II & Robotics
% Author:  Hasan Abuzaid
% University: German Jordanian University (GJU)
%
% This script:
%   1. Defines the linearized state-space model
%   2. Verifies open-loop instability
%   3. Designs an LQR optimal controller
%   4. Proves closed-loop stability via eigenvalue analysis
%   5. Simulates the step response from a 15 cm initial offset
% ============================================================

clc; clear; close all;

%% 1. PHYSICAL PARAMETERS
g      = 9.81;                       % Gravitational acceleration [m/s^2]
m_ball = 0.0027;                     % Mass of ping pong ball [kg]
r_ball = 0.020;                      % Radius of ball [m]
c      = 5/7;                        % Solid sphere rolling constant

%% 2. STATE-SPACE MODEL
% State vector: x = [position; velocity; plate_angle; angular_velocity]
% Input u: angular acceleration of plate

A = [0  1  0    0;
     0  0  c*g  0;
     0  0  0    1;
     0  0  0    0];

B = [0; 0; 0; 1];
C = [1  0  0  0];
D = 0;

%% 3. OPEN-LOOP STABILITY
poles_open = eig(A);

fprintf('================================================\n');
fprintf('  OPEN-LOOP STABILITY\n');
fprintf('================================================\n');
fprintf('  Poles: ');
fprintf('%.4f  ', poles_open);
fprintf('\n');

if any(real(poles_open) > 0)
    fprintf('  UNSTABLE (poles in RHP)\n');
else
    fprintf('  MARGINALLY UNSTABLE (poles on imaginary axis)\n');
end
fprintf('\n');

%% 4. LQR CONTROLLER DESIGN
Q = diag([50, 1, 1, 1]);
R = 1;

[K, S, P] = lqr(A, B, Q, R);

fprintf('================================================\n');
fprintf('  LQR GAINS\n');
fprintf('================================================\n');
fprintf('  K = [%.4f,  %.4f,  %.4f,  %.4f]\n', K(1), K(2), K(3), K(4));
fprintf('\n');

%% 5. CLOSED-LOOP STABILITY
A_closed = A - B*K;
poles_closed = eig(A_closed);

fprintf('================================================\n');
fprintf('  CLOSED-LOOP STABILITY\n');
fprintf('================================================\n');
fprintf('  Poles: ');
fprintf('%.4f%+.4fi  ', [real(poles_closed)'; imag(poles_closed)']);
fprintf('\n');

if all(real(poles_closed) < 0)
    fprintf('  STABLE (all poles in LHP)\n');
else
    fprintf('  UNSTABLE — retune Q/R\n');
end
fprintf('\n');

%% 6. SIMULATION
sys_cl = ss(A_closed, B*K, C, D);
x0 = [0.15; 0; 0; 0];   % Ball starts 15 cm from center
t  = 0:0.01:4;

[y, t, x] = initial(sys_cl, x0, t);

%% 7. PLOTTING
figure('Name', 'Ball & Plate LQR Simulation', 'Position', [100 100 900 600]);

subplot(2, 1, 1);
plot(t, x(:,1) * 100, 'LineWidth', 2, 'Color', [0.2 0.4 0.8]);
hold on; yline(0, '--k', 'LineWidth', 1); hold off;
grid on;
title('Ball Position Response', 'FontSize', 14);
xlabel('Time [s]'); ylabel('Position [cm]');
legend('Ball Position', 'Target (Center)', 'Location', 'northeast');

subplot(2, 1, 2);
plot(t, rad2deg(x(:,3)), 'LineWidth', 2, 'Color', [0.8 0.2 0.2]);
grid on;
title('Plate Angle (Control Effort)', 'FontSize', 14);
xlabel('Time [s]'); ylabel('Angle [deg]');

sgtitle('Ball & Plate — LQR Stability Proof', 'FontSize', 16, 'FontWeight', 'bold');
