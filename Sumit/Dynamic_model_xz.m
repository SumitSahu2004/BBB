function dq = dynamics_xz(t, x, u, param)
% x = [xk ; θy ; xk_dot ; θy_dot]
% u = torque around x-axis (virtual torque τx)

% Extract states
xk     = x(1);
th     = x(2);
xk_dot = x(3);
th_dot = x(4);

%% Parameters
mk = param.mk;     % ball mass
Ik = param.Ik;     % ball inertia
ma = param.ma;     % body mass
Iy = param.Iy;     % body inertia about y
l  = param.l;      % COM height
a  = param.a;      % zenith angle (rad)
rk = param.rk;     % ball radius
bx = param.bx;     % viscous friction ground
d_x = param.dx;    % Coulomb friction ground
bry = param.bry;   % friction between ball-body
dry = param.dry;   % Coulomb between ball-body
g  = 9.81;

%% Mass matrix M(q)
M11 = mk + Ik/(rk^2) + (ma+Ik)*cos(a)^2;
M12 = (ma*l+Ik)*cos(a);
M21 = M12;
M22 = Iy + ma*l^2;

M = [M11 M12;
     M21 M22];

%% C(q,q̇)
C = [0 , -ma*l*sin(a)*th_dot;
     0 , 0];

%% Gravity G(q)
G = [0;
     -ma*g*l*sin(th)];

%% Friction D(q̇)
Dx = bx*xk_dot + d_x*sign(xk_dot);
Dy = bry*th_dot + dry*sign(th_dot);
D  = [Dx; Dy];

%% Input Q
Q = [u/rk; 0];  % torque acts on ball only

%% Solve q̈
qdd = M \ (Q - C*[xk_dot; th_dot] - D - G);

%% Output state derivative
dq = [xk_dot;
      th_dot;
      qdd(1);
      qdd(2)];
end
