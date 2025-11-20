% ball_parameters.m
% Physical parameters (values taken/estimated from paper Table 1)
% See paper for derivations and more exact parameter estimation. :contentReference[oaicite:9]{index=9}

params = struct();

% Body (vehicle) parameters (paper Table 1)
params.m_body = 68.0;      % kg (mass of the body)
params.l = 0.38;           % m  (height of COM above ball center)
params.Ix = 12.1;          % kg*m^2 (moment of inertia about x)
params.Iy = 11.67;         % kg*m^2 (moment of inertia about y)
params.Iz = 1.08;          % kg*m^2 (moment of inertia about z)

% Ball / wheel parameters
params.m_ball = 19.6;      % kg
params.rk = 0.22;          % m (ball radius)
params.Ik = 0.38;          % kg*m^2 (moment of inertia of ball)

% Omni-wheel parameters (approx / paper)
params.rw = 0.1;           % m (radius of omni wheel)
params.Iw = 0.26;          % kg*m^2 (moment of inertia wheel)
params.alpha_deg = 65.5;   % zenith angle (deg)
params.alpha = params.alpha_deg*pi/180;

% Friction / damping estimates (paper uses viscous + coulomb)
params.by = 0.075; params.brx = 0.65;  % viscous damping (example from paper)
params.dx = 0.25; params.drx = 0.21;   % coulomb friction approximations

% Motor model simplified parameters (paper linearizes motor)
params.k_t = 0.1;  % torque constant (example)
params.k_e = 0.1;  % back-emf const (example)
params.Rm  = 1.0;  % motor resistance approximation

% Simulation settings
params.dt = 0.015;    % control interval used in paper (15 ms). :contentReference[oaicite:10]{index=10}
params.Tsim = 20;     % total simulation time (s)
