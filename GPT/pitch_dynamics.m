function dx_p_dt = pitch_dynamics(t, x_p, tau_y)
%% pitch_dynamics 2-DOF X-Z plane (Simulink Data Store friendly)
% x_p = [x_k ; theta_y ; x_k_dot ; theta_y_dot]
m_a = m_a; m_k = m_k; l = l; I_y = I_y; I_w = I_w;
r_k = r_k; r_w = r_w; alpha_rad = alpha_rad; g = g; I_k_param = I_k_param;

x_k = x_p(1); theta_y = x_p(2); x_k_dot = x_p(3); theta_y_dot = x_p(4);
q_y_dot = [x_k_dot; theta_y_dot];

M11 = m_k + (I_k_param / r_k^2) + m_a + (3*I_w*cos(alpha_rad)^2) / (2*r_w^2);
M12 =  m_a*l*cos(theta_y) - (3*I_w*cos(alpha_rad)^2)/(2*r_w^2) * r_k;
M21 = M12;
M22 = m_a*l^2 + (3*I_w*r_k^2*cos(alpha_rad)^2)/(2*r_w^2) + I_y;
M_y = [M11 M12; M21 M22];

C_y = [0, -m_a*l*theta_y_dot*sin(theta_y); 0, 0];
G_y = [0; -m_a*g*l*sin(theta_y)];
D_y = [0; 0];
Q_y = [ -(1/r_w)*tau_y ; (r_k/r_w)*tau_y ];

q_y_ddot = M_y \\ (Q_y - C_y*q_y_dot - D_y - G_y);
dx_p_dt = [ x_k_dot; theta_y_dot; q_y_ddot(1); q_y_ddot(2) ];
end
