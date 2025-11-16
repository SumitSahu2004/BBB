function dx_z_dt = yaw_dynamics(t, x_z, tau_z)
%% yaw_dynamics 1-DOF X-Y plane (Simulink Data Store friendly)
% x_z = [theta_z ; theta_z_dot]

I_k_param = I_k_param; I_z = I_z; I_w = I_w; r_k = r_k; r_w = r_w; alpha_rad = alpha_rad;

theta_z = x_z(1); theta_z_dot = x_z(2);

num = I_k_param * (r_k / r_w) * tau_z;
den = I_k_param * I_z + 3 * (I_k_param + I_z) * I_w * (r_k / r_w)^2 * sin(alpha_rad)^2;
theta_z_ddot = num / den;

dx_z_dt = [ theta_z_dot; theta_z_ddot ];
end
