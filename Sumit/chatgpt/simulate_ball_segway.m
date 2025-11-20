% simulate_ball_segway.m
% Top-level runner — sets up parameters, designs LQR, runs simulation, calls plotting.
% This script executes the double-loop control in simulation and logs results.

clear; close all; clc;

% load parameters
params = []; run('ball_parameters.m'); % populates 'params' variable
% If ball_parameters.m returns 'params' as struct, adjust above accordingly.
% (We wrote ball_parameters to return 'params' variable.)

% Build linear model (for LQR design)
[A,B] = ball_linear_model(params);

% LQR design (paper suggests Q,R choices; adjust as needed) :contentReference[oaicite:13]{index=13}
Q = diag([10,10,4,4,1]);
R = 4;  % scalar or small matrix

% compute LQR gain (5x5 -> control scalar)
K_lqr = lqr(A,B,Q,R);

% The paper shows Kr and Kp vectors for roll/pitch (example) — we include them as defaults
% Use the example gains from the paper if you want to match their sim results. :contentReference[oaicite:14]{index=14}
Kr = [-6.98, 4.45, 20.88, 6.09, 5.11];  % example row
Kp = [-3.27, 4.01, 29.07, 8.54, 5.72];  % example row

% Choose which to use: computed K_lqr rows or paper numbers:
gains.Kr = Kr_paper;    % outer-loop roll
gains.Kp = Kp_paper;    % outer-loop pitch

% PI inner-loop gains from paper (Eq. 35/41/42)
gains.Kpi.Kpr = 6;
gains.Kpi.Kir = 36;
gains.Kpi.Kpp = 6;
gains.Kpi.Kip = 36;

% feedforward compensation initial
gains.uxf = 0;
gains.uyf = 0;

% Integrators initial
integrators.roll = 0;
integrators.pitch = 0;

% Simulation initialization
dt = params.dt;
Tsim = params.Tsim;
N = round(Tsim/dt);
time = (0:N-1)*dt;

% initialize state: 5 element augmented state per plane (we simulate combined-ish)
x = zeros(5,1);  % start near zero (upright)
% set small initial tilt to test balance
x(2) = deg2rad(2);  % 2 degree pitch/roll tilt

% logs
X = zeros(5,N);
U = zeros(3,N); % motor torques (3 motors)
Vref = zeros(2,N);

for k = 1:N
    % call controller (we pass same x to both roll/pitch in simplified sim)
    [tau1, tau2, logs] = lqr_pi_controller(x, params, gains, integrators, dt);

    % Update integrators (controller returned integrators inside logs)
    integrators = logs.integrators;

    % Apply simple linear dynamics update: xdot = A*x + B*u (we map ux_eq as input)
    % Use ux_eq from logs (equivalent torque) as scalar control for the linear model
    u_eq = logs.ux_eq;  % for demonstration use ux; for full 2D you'd use both ux,uy
    xdot = A*x + B * u_eq;
    x = x + xdot * dt;

    % Save logs
    X(:,k) = x;
    U(1:3,k) = [logs.motor_tau; 0];
    Vref(:,k) = [logs.vx_ref; logs.vy_ref];
end

% Save to workspace for plotting
simdata.time = time;
simdata.X = X;
simdata.U = U;
simdata.Vref = Vref;

% Plot
plot_results(simdata);

