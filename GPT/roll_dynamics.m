function dx_r_dt = roll_dynamics(t, x_r, tau_x)
%% roll_dynamics 2-DOF Y-Z plane (Simulink Data Store friendly)
% x_r = [y_k ; theta_x ; y_k_dot ; theta_x_dot]

% Read data store variables (Data Store Memory blocks must exist in model)
m_a = m_a; m_k = m_k; l = l; I_x = I_x; I_w = I_w;
r_k = r_k; r_w = r_w; alpha_rad = alpha_rad; g = g; I_k_param = I_k_param;

y_k = x_r(1); theta_x = x_r(2); y_k_dot = x_r(3); theta_x_dot = x_r(4);
q_x_dot = [y_k_dot; theta_x_dot];

% Mass matrix
M11 = m_k + (I_k_param / r_k^2) + m_a + (3*I_w*cos(alpha_rad)^2) / (2*r_w^2);
M12 = (3*I_w*cos(alpha_rad)^2)/(2*r_w^2) * r_k - m_a*l*cos(theta_x);
M21 = M12;
M22 = m_a*l^2 + (3*I_w*r_k^2*cos(alpha_rad)^2)/(2*r_w^2) + I_x;
M_x = [M11 M12; M21 M22];

% Coriolis
C_x = [0, m_a*l*theta_x_dot*sin(theta_x); 0, 0];

% Gravity
G_x = [0; -m_a*g*l*sin(theta_x)];

% Friction (set zero here)
D_x = [0;0];

% Input mapping
Q_x = [ (1/r_w)*tau_x ; (r_k/r_w)*tau_x ];

% Accelerations
q_x_ddot = M_x \\ (Q_x - C_x*q_x_dot - D_x - G_x);

dx_r_dt = [ y_k_dot; theta_x_dot; q_x_ddot(1); q_x_ddot(2) ];
end
