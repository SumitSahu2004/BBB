function params = ParametersBlock()
%#codegen
% ParametersBlock — Outputs parameter structure for Simulink

params.m_a       = 68;
params.m_k       = 19.6;
params.l         = 0.38;
params.I_x       = 12.1;
params.I_y       = 11.67;
params.I_z       = 1.08;
params.r_w       = 0.1;
params.I_w       = 0.26;
params.r_k       = 0.22;
params.I_k       = 0.38;
params.alpha     = 65.5;
params.alpha_rad = 65.5 * (pi/180);
params.g         = 9.81;
end
