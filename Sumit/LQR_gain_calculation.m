% ========================================================================
%   LQR GAIN CALCULATION SCRIPT
%   Run this script ONCE from the MATLAB command window before
%   starting your Simulink simulation.
%
%   It will calculate the gain matrices 'Kr' and 'Kp' and
%   put them in the Base Workspace for Simulink to use.
% ========================================================================
clear; clc;

%% ------------------------------------------------------------
% DEFINE PARAMETERS
% (These would come from your 'params_vec' in the model)
%% ------------------------------------------------------------
% --- FAKE TEST DATA ---
% You MUST replace these with your actual parameter values!
mk  = 19.6;                   % (1) ball mass
Ik  = 0.38;                   % (2) ball inertia
ma  = 68;                     % (3) body mass
Ix  = 12.1;                   % (4) body inertia x
Iy  = 11.67;                  % (5) body inertia y
Iz  = 1.08;                   % (6) body inertia z
l   = 0.38;                   % (7) COM height
alpha   = 65.5;          % (8) zenith angle
rk  = 0.22;                   % (9) ball radius
rw  = 0.10;                   % (10) omniwheel radius
bx  = 0.10;                   % (11) viscous friction xz
dx  = 0.01;                   % (12) coulomb friction xz
by  = 0.10;                   % (13) viscous friction yz
dy  = 0.01;                   % (14) coulomb friction yz
bry = 0.12;                   % (15) body/ball viscous xz
dry = 0.02;                   % (16) coulomb xz
brx = 0.12;                   % (17) body/ball viscous yz
drx = 0.02;                   % (18) coulomb yz
g   = 9.81;                   % (19) gravity
k_motor = 1;                  % (20) motor constant

% --- END FAKE DATA ---

% --- Values from your function ---
alpha_rad = deg2rad(alpha);
Iw = 0.26; % Value you added
K_pr = 6;
K_pp = 6;
K_ir = 36;
K_ip = 36;

% --- Useful repeated terms (copied from your code) ---
mau_r = (2*(rw^2)*Ik*(Iy + ma*l^2)) + 3*Iw*(ma+mk)*(rk^4)*(cos(alpha_rad))^2 ...
        + 2*Ix*(ma+mk)*(rk^2)*(rw^2) + 3*(Ik+Ix+2*ma*l*rk)*(Iw*rk^2)*(cos(alpha_rad))^2 ...
        + 2*ma*mk*l^2*(rk^2)*(rw^2) + 3*Iw*ma*(l^2)*(rk^2)*(cos(alpha_rad))^2;

mau_p = (2*(rw^2)*Ik*(Iy+ma*(l^2))) + 3*Iw*(ma+mk)*(rk^4)*((cos(alpha_rad))^2) ...
+ 2*Iy*(ma+mk)*(rk^2)*(rw^2) + 3*(Ik+Iy+2*ma*l*rk)*Iw*(rk^2)*((cos(alpha_rad))^2) ...
+ 2*ma*mk*(l^2)*(rk^2)*((cos(alpha_rad))^2);

%% ------------------------------------------------------------
% BUILD LINEARIZED MATRICES FOR ROLL (Ar, Br)
% (Copied from your code, with lx -> Ix fixed)
%% ------------------------------------------------------------
Ar = zeros(5,5);
Br = zeros(5,1);
Ar(1,3) = 1;
Ar(2,4) = 1;
Ar(3,1) = -1*K_ir*(rk^2)*rw*(ma*(l^2) + ma*rk*l + Ix) / mau_r; % Fixed lx
Ar(3,2) =  ma*g*l*(rk^2)*(2*ma*l*(rw^2) - 3*Iw*rk*((cos(alpha_rad))^2))/ mau_r;
Ar(3,3) = -rk^2 * (2*(Ix + ma*l^2)*(K_pr*rw + by*(rw^2)) + 3*Iw*by*(rk^2)*((cos(alpha_rad))^2) + 2*K_pr*ma*l*rk*rw) / mau_r;
Ar(3,4) = -1*brx*(2*ma*l*(rw^2) - 3*Iw*rk*((cos(alpha_rad))^2))/ mau_r;
Ar(3,5) = K_ir*(rk^2)*rw*(ma*(l^2) + ma*mk*l +Ix)/ mau_r;
Ar(4,1) = -2*K_ir*rk*rw*(Ik + (ma + mk)*(rk^2) + ma*l*rk) / mau_r;
Ar(4,2) = ma*g*l*(2*Ik*(rw^2) + 2*(ma + mk)*(rk^2)*(rw^2) + ma*l*rk) / mau_r;
Ar(4,3) = -rk*(2*K_pr*rw*(Ik + ma*(rk^2) + mk*(rk^2) + ma*l*rk) - 3*Iw*by*(rk^2)*((cos(alpha_rad))^2) + 2*by*ma*l*rk*(rw^2)) / mau_r;
Ar(4,4) = -1*brx*(2*Ik*(rw^2) + 2*(ma + mk)*(rk^2) + 3*Iw*(rk^2)*((cos(alpha_rad))^2)) / mau_r;
Ar(4,5) = 2*K_ir*rk*rw*(Ik + (ma + mk)*(rk^2) + ma*l*rk) / mau_r;
% B_r
Br(3) = (2*K_pr*(rk^2)*rw*(ma*(l^2) + ma*l*rk + Ix)) / mau_r; % Fixed lx
Br(4) = 2*K_pr*rk*rw*(Ik + (ma + mk)*(rk^2) + ma*l*rk)/mau_r;
Br(5) = 1;

%% ------------------------------------------------------------
% BUILD LINEARIZED MATRICES FOR PITCH (Ap, Bp)
% (Copied from your code, with ly -> Iy fixed)
%% ------------------------------------------------------------
Ap = zeros(5,5);
Bp = zeros(5,1);
Ap(1,3) = 1;
Ap(2,4) = 1;
% **FIXED**: Restored the "-1 *" here, which was accidentally deleted
Ap(3,1) = -1*K_ip*(rk^2)*rw*(ma*(l^2) + ma*rk*l + Iy) / mau_p; % Fixed ly
Ap(3,2) =  ma*g*l*(rk^2)*(2*ma*l*(rw^2) - 3*Iw*rk*((cos(alpha_rad))^2))/ mau_p;
Ap(3,3) = 2*rk^2 * (Iy*K_pp*rw - Iy*bx*(rw^2 - 1.5*Iw*bx*(rk^2)*((cos(alpha_rad))^2)) + ma*(l^2)*rw*(K_pp-bx*rw) + K_pp*ma*l*rk*rw) / mau_p;
Ap(3,4) = bry*(rk^2)*(2*ma*l*(rw^2) - 3*Iw*rk*((cos(alpha_rad))^2))/ mau_p;
Ap(3,5) = -K_ip*(rk^2)*rw*(ma*(l^2) + ma*rk*l +Iy)/ mau_p; % Note: This term is different from Ap(3,1), verify if correct
Ap(4,1) = -2*K_ip*rk*rw*(Ik + (ma + mk)*(rk^2) + ma*l*rk) / mau_p;
Ap(4,2) = ma*g*l*(2*Ik*(rw^2) + 2*(ma + mk)*(rk^2)*(rw^2) + 3*Iw*(rk^2)*((cos(alpha_rad))^2)) / mau_p;
Ap(4,3) = -rk*(2*K_pp*rw*(Ik + ma*(rk^2) + mk*(rk^2) + ma*l*rk) + 3*Iw*by*(rk^2)*((cos(alpha_rad))^2) - 2*by*ma*l*rk*(rw^2)) / mau_p;
Ap(4,4) = -1*bry*(2*Ik*(rw^2) + 2*(ma + mk)*(rk^2) + 3*Iw*(rk^2)*((cos(alpha_rad))^2)) / mau_p;
Ap(4,5) = 2*K_ip*rk*rw*(Ik + (ma + mk)*(rk^2) + ma*l*rk) / mau_p;
% B_p
Bp(3) = (2*K_pp*(rk^2)*rw*(ma*(l^2) + ma*l*rk + Iy)) / mau_p; % Fixed ly
Bp(4) = 2*K_pp*rk*rw*(Ik + (ma + mk)*(rk^2) + ma*l*rk)/mau_p;
Bp(5) = 1;

%% ------------------------------------------------------------
% Q & R MATRICES
%% ------------------------------------------------------------
Qr = diag([10 2 10 2 4]);
Rr = 4;
Qp = diag([10 5 10 5 5]);
Rp = 2;

%% ------------------------------------------------------------
% LQR GAINS (This is where 'lqr' is safely called)
%% ------------------------------------------------------------
Kr = lqr(Ar, Br, Qr, Rr);
Kp = lqr(Ap, Bp, Qp, Rp);

disp('Gains Kr and Kp calculated and are in the workspace.');
disp('You can now run your Simulink model.');