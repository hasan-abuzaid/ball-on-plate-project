%% BALL AND PLATE — FULL PROJECT ANALYSIS (Real Parameters)
% ============================================================
% Course:  Control Systems II & Robotics
% Author:  Hasan Abuzaid
% University: German Jordanian University (GJU)
%
% Uses the actual physical parameters from the built system:
%   - Ball mass: 0.002 kg
%   - Ball radius: 0.04 m
%   - Plate: 30x30 cm
%   - Servo arm: 5.5 cm
%   - Rod length: 11 cm
%
% Includes:
%   - Controllability analysis
%   - LQR with project-tuned Q matrix
%   - Reference scaling (Nbar) for zero steady-state error
%   - Closed-loop step response
% ============================================================

clc; clear; close all;

%% 1. PHYSICAL PARAMETERS (Real Project Values)
g       = 9.81;          % Gravity [m/s^2]
m       = 0.002;         % Ball mass [kg]
R       = 0.04;          % Ball radius [m]
L_plate = 0.30;          % Plate length [m]
d_servo = 0.055;         % Servo arm length [m]
L_rod   = 0.11;          % Linkage rod length [m]

% Ball inertia (solid sphere model)
J = (2/5) * m * R^2;

% Linearized coupling constant
H = -m * g / (J / R^2 + m);

%% 2. STATE-SPACE MODEL
% States: x = [position; velocity; plate_angle; angular_velocity]
% Input:  u = angular acceleration of plate

A = [0  1  0  0;
     0  0  H  0;
     0  0  0  1;
     0  0  0  0];

B = [0; 0; 0; 1];
C = [1  0  0  0];
D = 0;

fprintf('================================================\n');
fprintf('  SYSTEM PARAMETERS\n');
fprintf('================================================\n');
fprintf('  Ball mass:     %.4f kg\n', m);
fprintf('  Ball radius:   %.4f m\n', R);
fprintf('  Coupling H:    %.4f\n', H);
fprintf('\n');

%% 3. CONTROLLABILITY CHECK
Co = ctrb(A, B);
controllability_rank = rank(Co);

fprintf('================================================\n');
fprintf('  CONTROLLABILITY\n');
fprintf('================================================\n');
fprintf('  Controllability matrix rank: %d / %d\n', controllability_rank, size(A, 1));

if controllability_rank == size(A, 1)
    fprintf('  FULLY CONTROLLABLE\n');
else
    fprintf('  NOT FULLY CONTROLLABLE — check model\n');
end
fprintf('\n');

%% 4. OPEN-LOOP STABILITY
poles_open = eig(A);

fprintf('================================================\n');
fprintf('  OPEN-LOOP STABILITY\n');
fprintf('================================================\n');
fprintf('  Poles: ');
fprintf('%.4f  ', poles_open);
fprintf('\n');
fprintf('  MARGINALLY UNSTABLE (poles at origin)\n\n');

%% 5. LQR DESIGN (Project-Tuned Weights)
% Heavier penalties than the basic analysis to match real hardware behavior
Q = diag([300, 20, 150, 2]);
R_cost = 1;

[K_lqr, S, P] = lqr(A, B, Q, R_cost);

fprintf('================================================\n');
fprintf('  LQR GAINS (Project Tuning)\n');
fprintf('================================================\n');
fprintf('  K = [%.4f,  %.4f,  %.4f,  %.4f]\n', K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4));
fprintf('\n');

%% 6. REFERENCE SCALING (Nbar)
% Ensures zero steady-state error for step reference tracking
% u = -Kx + Nbar * r

% Compute Nbar using the standard formula
sys_ss = ss(A, B, C, D);
Nbar = rscale(A, B, C, D, K_lqr);

fprintf('================================================\n');
fprintf('  REFERENCE SCALING\n');
fprintf('================================================\n');
fprintf('  Nbar = %.4f\n', Nbar);
fprintf('  Control law: u = -Kx + Nbar * r\n\n');

%% 7. CLOSED-LOOP STABILITY
A_cl = A - B * K_lqr;
poles_closed = eig(A_cl);

fprintf('================================================\n');
fprintf('  CLOSED-LOOP STABILITY\n');
fprintf('================================================\n');
fprintf('  Poles:\n');
for i = 1:length(poles_closed)
    fprintf('    p%d = %.4f %+.4fi\n', i, real(poles_closed(i)), imag(poles_closed(i)));
end

if all(real(poles_closed) < 0)
    fprintf('  ASYMPTOTICALLY STABLE\n');
else
    fprintf('  UNSTABLE — retune\n');
end
fprintf('\n');

%% 8. STEP RESPONSE SIMULATION
% Closed-loop system with reference input
sys_cl = ss(A_cl, B * Nbar, C, D);
t = 0:0.001:5;

% Step response: command ball to move 10 cm from origin
[y, t] = step(sys_cl, t);
y = y * 0.10;  % Scale to 10 cm reference

% Initial condition response: ball starts at 15 cm offset
sys_ic = ss(A_cl, B * K_lqr, C, D);
x0 = [0.15; 0; 0; 0];
[y_ic, t_ic, x_ic] = initial(sys_ic, x0, t);

%% 9. PLOTTING
figure('Name', 'Ball & Plate — Full Project Analysis', 'Position', [50 50 1000 700]);

% Step response
subplot(2, 2, 1);
plot(t, y * 100, 'b', 'LineWidth', 2);
hold on; yline(10, '--r', 'LineWidth', 1); hold off;
grid on;
title('Step Response (10 cm Reference)');
xlabel('Time [s]'); ylabel('Position [cm]');
legend('Response', 'Reference', 'Location', 'southeast');

% Initial condition response
subplot(2, 2, 2);
plot(t_ic, x_ic(:,1) * 100, 'b', 'LineWidth', 2);
hold on; yline(0, '--k', 'LineWidth', 1); hold off;
grid on;
title('Disturbance Rejection (15 cm offset)');
xlabel('Time [s]'); ylabel('Position [cm]');
legend('Ball Position', 'Target', 'Location', 'northeast');

% Plate angle during recovery
subplot(2, 2, 3);
plot(t_ic, rad2deg(x_ic(:,3)), 'r', 'LineWidth', 2);
grid on;
title('Plate Angle During Recovery');
xlabel('Time [s]'); ylabel('Angle [deg]');

% Pole-zero map
subplot(2, 2, 4);
plot(real(poles_open), imag(poles_open), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
plot(real(poles_closed), imag(poles_closed), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
xline(0, '--k');
hold off;
grid on;
title('Pole Migration (Open → Closed Loop)');
xlabel('Real'); ylabel('Imaginary');
legend('Open-Loop', 'Closed-Loop', 'Location', 'best');

sgtitle('Ball & Plate — Complete Analysis', 'FontSize', 16, 'FontWeight', 'bold');


%% === HELPER FUNCTION ===
function Nbar = rscale(A, B, C, D, K)
    % Compute reference scaling gain for zero steady-state error
    s = size(A, 1);
    Z = [zeros([1, s]) 1];
    N = inv([A, B; C, D]) * Z';
    Nx = N(1:s);
    Nu = N(1+s);
    Nbar = Nu + K * Nx;
end
