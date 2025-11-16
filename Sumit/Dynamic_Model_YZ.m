function dq = dynamics_yz(t, x, u, param)

yk     = x(1);
th     = x(2);
yk_dot = x(3);
th_dot = x(4);

%% Parameters
mk = param.mk;
Ik = param.Ik;
ma = param.ma;
Ix = param.Ix;
l  = param.l;
a  = param.a;
rk = param.rk;
by = param.by;
dy = param.dy;
brx = param.brx;
drx = param.drx;
g  = 9.81;

%% Mass matrix M(q)
M11 = mk + Ik/(rk^2) + ma*cos(a)^2;
M12 = (ma*l)*cos(a);
M21 = M12;
M22 = Ix + ma*l^2;

M = [M11 M12;
     M21 M22];

%% Coriolis term
C = [0, -ma*l*sin(a)*th_dot;
     0, 0];

%% Gravity
G = [0;
     -ma*g*l*sin(th)];

%% Friction
Dy = by*yk_dot + dy*sign(yk_dot);
Dth = brx*th_dot + drx*sign(th_dot);
D = [Dy; Dth];

%% Input torque
Q = [u/rk; 0];   % only ball actuator

%% Solve q̈
qdd = M \ (Q - C*[yk_dot; th_dot] - D - G);

%% Output derivative
dq = [yk_dot;
      th_dot;
      qdd(1);
      qdd(2)];
end
