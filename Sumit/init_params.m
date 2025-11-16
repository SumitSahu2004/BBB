function robot_parameters()
% GLOBAL PARAMETERS FOR BALL SEGWAY (based on the paper)

global mk Ik ma Ix Iy Iz l a rk rw ...
       bx dx by dy brx drx bry dry g

% ---- Ball ----
mk = 19.6;        % mass of ball
Ik = 0.38;        % inertia of ball (kg*m^2)
rk = 0.22;        % ball radius (m)

% ---- Body ----
ma = 68;          % mass of body
Ix = 12.1;        % inertia about x-axis
Iy = 11.67;       % inertia about y-axis
Iz = 1.08;        % inertia about z-axis
l  = 0.38;        % COM height

% ---- Omniwheels ----
rw = 0.1;         % wheel radius
a  = deg2rad(65.5); % zenith angle in radians

% ---- Friction ----
bx  = 0.045;  % viscous friction x-plane (approx)
dx  = 0.03;   % Coulomb friction
by  = 0.045;
dy  = 0.03;
brx = 0.025;  
drx = 0.015;
bry = 0.025;
dry = 0.015;

% ---- Gravity ----
g = 9.81;
end
