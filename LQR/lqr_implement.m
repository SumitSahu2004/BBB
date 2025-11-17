%% Ball Segway LQR Controller Design (Roll and Pitch Planes)
% This script calculates the optimal LQR gain matrices (Kr and Kp) for the 
% Ball Segway augmented system based on the linearized model (Ar, Br, Ap, Bp)
% and weighting matrices (Q, R) as detailed in the research paper: 
% "Balancing and Transferring Control of a Ball Segway Using a Double-Loop Approach."
%
% The LQR is the outer-loop controller. The state vectors are augmented to 
% include an integral error term (x_5r/x_5p) to ensure zero steady-state error.
% State Vector: x_a = [position, angle, velocity, angular_velocity, integral_error]'
% Control Input: v = reference speed (input to inner-loop PI controller)

clear; clc;

% --- 1. System Parameters (from Table 1, Page 6) ---
disp('1. Defining System Parameters...');
ma = 68;      % Mass of the body (kg)
l  = 0.38;     % Distance from CoM to ball center (m)
Ix = 12.1;    % Moment of inertia of the body about the x-axis (Roll, kg-m^2)
Iy = 11.67;   % Moment of inertia of the body about the y-axis (Pitch, kg-m^2)
rw = 0.1;     % Radius of the omnidirectional wheel (m)
Iw = 0.26;    % Moment of inertia of the omnidirectional wheel (kg-m^2)
mk = 19.6;    % Mass of the ball (kg)
rk = 0.22;    % Radius of the ball (m)
Ik = 0.38;    % Moment of inertia of the ball (kg-m^2)
alpha = 65.5; % Zenith angle of omni-wheel (degrees)
g = 9.81;     % Gravitational acceleration (m/s^2)

% Convert alpha to radians and calculate trig terms
alpha_rad = deg2rad(alpha);
cos_alpha = cos(alpha_rad);
cos_alpha_sq = cos_alpha^2;

% --- 2. Inner-Loop PI Controller Gains (from Page 30) ---
Kpr = 6;    % Proportional gain (Roll)
Kir = 36;   % Integral gain (Roll)
Kpp = 6;    % Proportional gain (Pitch)
Kip = 36;   % Integral gain (Pitch)

% --- 3. Friction Assumption ---
% Viscous friction (b_y, b_rx, b_x, b_ry) are set to zero for the LQR linear 
% model derivation, as their effects are compensated by the PI inner-loop 
% and feedforward terms (Page 14).
by = 0;   
brx = 0;
bx = 0;   
bry = 0;

%% --- ROLL PLANE (y-z axis) LQR CALCULATION ---
disp(' ');
disp('--- ROLL PLANE (y-z) LQR CALCULATION ---');

% --- 4a. Denominator Constant (mau_r) - Roll Plane (uses Ix) ---
% Based on the formula for mau_r on Page 15.
mau_r = (2*rw^2*Ik*(Ix + ma*l^2) ...
         + 3*Iw*(ma + mk)*rk^4*cos_alpha_sq ...
         + 2*Ix*(ma + mk)*rk^2*rw^2 ...
         + 3*(Ik + Ix + 2*ma*l*rk)*Iw*rk^2*cos_alpha_sq ...
         + 2*ma*mk*l^2*rk^2 ...
         + 3*Iw*ma*l^2*rk^2*cos_alpha_sq);
inv_mau_r = 1 / mau_r;

% --- 5a. Ar and Br Elements (Roll Plane, Page 15) ---
% Note: Terms containing 'by' or 'brx' will be zero based on the friction assumption.

% y_k_double_dot row (row 3)
a_r31_num = -Kir * rk^2 * rw * (ma*l^2 + ma*rk*l + Ix); 
a_r31 = a_r31_num * inv_mau_r;
a_r32_num = ma*g*l * rk^2 * (2*ma*l*rw^2 - 3*Iw*rk*cos_alpha_sq);
a_r32 = a_r32_num * inv_mau_r;
a_r33_num = -rk^2 * (2*(Ix + ma*l^2)*(Kpr*rw + by*rw^2) + 3*Iw*by*rk^2*cos_alpha_sq + 2*Kpr*ma*l*rk*rw);
a_r33 = a_r33_num * inv_mau_r;
a_r34_num = -brx * rk^2 * (2*ma*l*rw^2 - 3*Iw*rk*cos_alpha_sq);
a_r34 = a_r34_num * inv_mau_r;
a_r35_num = Kir * rk^2 * rw * (ma*l^2 + ma*rk*l + Ix); % Corrected based on structure similarity to a_r31
a_r35 = a_r35_num * inv_mau_r;

% theta_x_double_dot row (row 4)
a_r41_num = -2 * Kir * rk * rw * (Ik + (ma + mk)*rk^2 + ma*l*rk);
a_r41 = a_r41_num * inv_mau_r;
a_r42_num = ma*g*l * (2*Ik*rw^2 + 2*(ma + mk)*rk^2*rw^2 + 3*Iw*rk^2*cos_alpha_sq);
a_r42 = a_r42_num * inv_mau_r;
a_r43_num = -rk * (2*Kpr*rw*(Ik + ma*rk^2 + mk*rk^2 + ma*l*rk) - 3*Iw*by*rk^2*cos_alpha_sq + 2*by*ma*l*rk*rw^2);
a_r43 = a_r43_num * inv_mau_r;
a_r44_num = -brx * (2*Ik*rw^2 + 2*(ma + mk)*rk^2*rw^2 + 3*Iw*rk^2*cos_alpha_sq);
a_r44 = a_r44_num * inv_mau_r;
a_r45_num = 2 * Kir * rk * rw * (Ik + (ma + mk)*rk^2 + ma*l*rk);
a_r45 = a_r45_num * inv_mau_r;

% Control input v_y terms
b_r3_num = 2 * Kpr * rk^2 * rw * (ma*l^2 + ma*l*rk + Ix);
b_r3 = b_r3_num * inv_mau_r;
b_r4_num = 2 * Kpr * rk * rw * (Ik + (ma + mk)*rk^2 + ma*l*rk);
b_r4 = b_r4_num * inv_mau_r;

% Construct Ar and Br matrices
Ar = [
    0, 0, 1, 0, 0;
    0, 0, 0, 1, 0;
    a_r31, a_r32, a_r33, a_r34, a_r35;
    a_r41, a_r42, a_r43, a_r44, a_r45;
    0, 0, 0, 0, 0
];

Br = [0; 0; b_r3; b_r4; 1];

% --- 6a. LQR Weighting Matrices (from Page 30) ---
% Qra = diag(ξry, ξrθ, ξr\dot{y}, ξr\dot{θ}, ξr5)
Qra = diag([10, 2, 10, 2, 4]); % Weights for [yk, theta_x, yk_dot, theta_x_dot, x_5r]
Rra = 4;                       % Weight for control effort (v_y)

% --- 7a. Calculate Optimal LQR Gain Kr ---
[Kr, Sr, Er] = lqr(Ar, Br, Qra, Rra);
disp(' ');
disp('State Vector: [y_k, theta_x, y_k_dot, theta_x_dot, x_5r]');
disp('Optimal Roll Gain Kr:');
disp(Kr);
disp(['Paper Kr (for comparison): [-6.98 -4.45 20.88 6.09 5.11]']);

%% --- PITCH PLANE (x-z axis) LQR CALCULATION ---
disp(' ');
disp('--- PITCH PLANE (x-z) LQR CALCULATION ---');

% --- 4b. Denominator Constant (mau_p) - Pitch Plane (uses Iy) ---
% Based on the formula for mau_p on Page 15.
mau_p = (2*rw^2*Ik*(Iy + ma*l^2) ...
         + 3*Iw*(ma + mk)*rk^4*cos_alpha_sq ...
         + 2*Iy*(ma + mk)*rk^2*rw^2 ...
         + 3*(Ik + Iy + 2*ma*l*rk)*Iw*rk^2*cos_alpha_sq ...
         + 2*ma*mk*l^2*rk^2 ...
         + 3*Iw*ma*l^2*rk^2*cos_alpha_sq);
inv_mau_p = 1 / mau_p;

% --- 5b. Ap and Bp Elements (Pitch Plane, Page 15) ---
% Note: Terms containing 'bx' or 'bry' will be zero based on the friction assumption.

% x_k_double_dot row (row 3)
a_p31_num = Kip * rk^2 * rw * (ma*l^2 + ma*rk*l + Iy); 
a_p31 = a_p31_num * inv_mau_p;
a_p32_num = -ma*g*l * rk^2 * (2*ma*l*rw^2 - 3*Iw*rk*cos_alpha_sq);
a_p32 = a_p32_num * inv_mau_p;
a_p33_num = 2*rk^2 * (Iy*Kpp*rw - Iy*bx*rw^2 - 1.5*Iw*bx*rk^2*cos_alpha_sq ...
                    + ma*l^2*rw*(Kpp - bx*rw) + Kpp*ma*l*rk*rw);
a_p33 = a_p33_num * inv_mau_p;
a_p34_num = bry * rk^2 * (2*ma*l*rw^2 - 3*Iw*rk*cos_alpha_sq);
a_p34 = a_p34_num * inv_mau_p;
a_p35_num = -Kip * rk^2 * rw * (ma*l^2 + ma*rk*l + Iy); 
a_p35 = a_p35_num * inv_mau_p;

% theta_y_double_dot row (row 4)
a_p41_num = -2 * Kip * rk * rw * (Ik + (ma + mk)*rk^2 + ma*l*rk);
a_p41 = a_p41_num * inv_mau_p;
a_p42_num = ma*g*l * (2*Ik*rw^2 + 2*(ma + mk)*rk^2*rw^2 + 3*Iw*rk^2*cos_alpha_sq);
a_p42 = a_p42_num * inv_mau_p;
a_p43_num = -rk * (2*Kpp*rw*(Ik + ma*rk^2 + mk*rk^2 + ma*l*rk) + 3*Iw*bx*rk^2*cos_alpha_sq - 2*bx*ma*l*rk*rw^2);
a_p43 = a_p43_num * inv_mau_p;
a_p44_num = -bry * (2*Ik*rw^2 + 2*(ma + mk)*rk^2*rw^2 + 3*Iw*rk^2*cos_alpha_sq);
a_p44 = a_p44_num * inv_mau_p;
a_p45_num = 2 * Kip * rk * rw * (Ik + (ma + mk)*rk^2 + ma*l*rk);
a_p45 = a_p45_num * inv_mau_p;

% Control input v_x terms
b_p3_num = -Kpp * rk^2 * rw * (ma*l^2 + ma*l*rk + Iy);
b_p3 = b_p3_num * inv_mau_p;
b_p4_num = 2 * Kpp * rk * rw * (Ik + (ma + mk)*rk^2 + ma*l*rk);
b_p4 = b_p4_num * inv_mau_p;

% Construct Ap and Bp matrices
Ap = [
    0, 0, 1, 0, 0;
    0, 0, 0, 1, 0;
    a_p31, a_p32, a_p33, a_p34, a_p35;
    a_p41, a_p42, a_p43, a_p44, a_p45;
    0, 0, 0, 0, 0
];

Bp = [0; 0; b_p3; b_p4; 1];

% --- 6b. LQR Weighting Matrices (from Page 30) ---
% Qpa = diag(ξpx, ξpθ, ξp\dot{x}, ξp\dot{θ}, ξp5)
Qpa = diag([10, 5, 10, 5, 5]); % Weights for [xk, theta_y, xk_dot, theta_y_dot, x_5p]
Rpa = 2;                       % Weight for control effort (v_x)

% --- 7b. Calculate Optimal LQR Gain Kp ---
[Kp, Sp, Ep] = lqr(Ap, Bp, Qpa, Rpa);
disp(' ');
disp('State Vector: [x_k, theta_y, x_k_dot, theta_y_dot, x_5p]');
disp('Optimal Pitch Gain Kp:');
disp(Kp);
disp(['Paper Kp (for comparison): [-3.27 4.01 29.07 8.54 5.72]']);

disp(' ');
disp('--- End of LQR Gain Calculation ---');
% The final control laws (v_y and v_x) are applied as:
% v_y = -Kr * [y_k; theta_x; y_k_dot; theta_x_dot; x_5r]
% v_x = -Kp * [x_k; theta_y; x_k_dot; theta_y_dot; x_5p]

T_final = 50;

ic_r=[10;  %position
    0.256;  %angle
    61;  %velocity
    20;  %angular velocity
    0]; %integral error

ic_p=[20;  %position
    0.856;  %angle
    21;  %velocity
    40;  %angular velocity
    0]; %integral error



out=sim('LQR_imple');
t= out.tout;
x1_r= out.sim_x_r(:,1);
x2_r= out.sim_x_r(:,2);
u1_r= out.sim_u_r(:,1);

x1_p= out.sim_y_p(:,1);
x2_p= out.sim_y_p(:,2);
u1_p= out.sim_u_p(:,1);

%plot
figure
subplot(3,2,1)
%title(['Scenario',num2str(Scenario)])
plot(t,x1_r,'LineWidth',2)
grid on
legend('x1_r')
title('Position');

subplot(3,2,3)
plot(t,x2_r,'LineWidth',2)
grid on
legend('x2_r')
title('Velocity');

subplot(3,2,5)
plot(t,u1_r,'LineWidth',2)
grid on
legend('u1_r')
title('Control');

subplot(3,2,2)
%title(['Scenario',num2str(Scenario)])
plot(t,x1_p,'LineWidth',2)
grid on
legend('x1_p')
title('Position');

subplot(3,2,4)
plot(t,x2_p,'LineWidth',2)
grid on
legend('x2_p')
title('Velocity');

subplot(3,2,6)
plot(t,u1_p,'LineWidth',2)
grid on
legend('u1_p')
title('Control');
