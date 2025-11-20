% demo script (run in command window)
params = [];                      % use defaults in function
X0 = zeros(14,1);                 % start upright zero states
X0(3) = deg2rad(2);               % small initial roll
X0(4) = deg2rad(1.5);             % small initial pitch

% constant virtual torque (zero = just test free stabilization of model)
u_fun = @(t) [0; 0; 0];

% wrapper to pass u into ODE45
odefun = @(t,x) ballsegway_dynamics(t,x, u_fun(t), params);

tspan = [0 8];
[tt, XX] = ode45(odefun, tspan, X0);

% quick plot of tilt angles
figure; plot(tt, rad2deg(XX(:,3))); hold on; plot(tt, rad2deg(XX(:,4)));
legend('theta_x (deg)','theta_y (deg)'); xlabel('Time (s)'); grid on;
