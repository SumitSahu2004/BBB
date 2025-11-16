function build_ballsegway_simulink()
% BUILD_BALLSEGWAY_SIMULINK
% Creates dynamics .m files and builds a Simulink model BallSegwayModel.slx
% The model uses Data Store Memory for parameters (no 'global' in MATLAB Function blocks).

%% 0. housekeeping
modelName = 'BallSegwayModel';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

%% 1. Create MATLAB function files (roll, pitch, yaw)
fprintf('Writing dynamics .m files...\n');

% --- roll_dynamics.m
rollCode = [
"function dx_r_dt = roll_dynamics(t, x_r, tau_x)"
"%% roll_dynamics 2-DOF Y-Z plane (Simulink Data Store friendly)"
"% x_r = [y_k ; theta_x ; y_k_dot ; theta_x_dot]"
""
"% Read data store variables (Data Store Memory blocks must exist in model)"
"m_a = m_a; m_k = m_k; l = l; I_x = I_x; I_w = I_w;"
"r_k = r_k; r_w = r_w; alpha_rad = alpha_rad; g = g; I_k_param = I_k_param;"
""
"y_k = x_r(1); theta_x = x_r(2); y_k_dot = x_r(3); theta_x_dot = x_r(4);"
"q_x_dot = [y_k_dot; theta_x_dot];"
""
"% Mass matrix"
"M11 = m_k + (I_k_param / r_k^2) + m_a + (3*I_w*cos(alpha_rad)^2) / (2*r_w^2);"
"M12 = (3*I_w*cos(alpha_rad)^2)/(2*r_w^2) * r_k - m_a*l*cos(theta_x);"
"M21 = M12;"
"M22 = m_a*l^2 + (3*I_w*r_k^2*cos(alpha_rad)^2)/(2*r_w^2) + I_x;"
"M_x = [M11 M12; M21 M22];"
""
"% Coriolis"
"C_x = [0, m_a*l*theta_x_dot*sin(theta_x); 0, 0];"
""
"% Gravity"
"G_x = [0; -m_a*g*l*sin(theta_x)];"
""
"% Friction (set zero here)"
"D_x = [0;0];"
""
"% Input mapping"
"Q_x = [ (1/r_w)*tau_x ; (r_k/r_w)*tau_x ];"
""
"% Accelerations"
"q_x_ddot = M_x \\ (Q_x - C_x*q_x_dot - D_x - G_x);"
""
"dx_r_dt = [ y_k_dot; theta_x_dot; q_x_ddot(1); q_x_ddot(2) ];"
"end"
];
write_block_file('roll_dynamics.m', rollCode);

% --- pitch_dynamics.m
pitchCode = [
"function dx_p_dt = pitch_dynamics(t, x_p, tau_y)"
"%% pitch_dynamics 2-DOF X-Z plane (Simulink Data Store friendly)"
"% x_p = [x_k ; theta_y ; x_k_dot ; theta_y_dot]"
"m_a = m_a; m_k = m_k; l = l; I_y = I_y; I_w = I_w;"
"r_k = r_k; r_w = r_w; alpha_rad = alpha_rad; g = g; I_k_param = I_k_param;"
""
"x_k = x_p(1); theta_y = x_p(2); x_k_dot = x_p(3); theta_y_dot = x_p(4);"
"q_y_dot = [x_k_dot; theta_y_dot];"
""
"M11 = m_k + (I_k_param / r_k^2) + m_a + (3*I_w*cos(alpha_rad)^2) / (2*r_w^2);"
"M12 =  m_a*l*cos(theta_y) - (3*I_w*cos(alpha_rad)^2)/(2*r_w^2) * r_k;"
"M21 = M12;"
"M22 = m_a*l^2 + (3*I_w*r_k^2*cos(alpha_rad)^2)/(2*r_w^2) + I_y;"
"M_y = [M11 M12; M21 M22];"
""
"C_y = [0, -m_a*l*theta_y_dot*sin(theta_y); 0, 0];"
"G_y = [0; -m_a*g*l*sin(theta_y)];"
"D_y = [0; 0];"
"Q_y = [ -(1/r_w)*tau_y ; (r_k/r_w)*tau_y ];"
""
"q_y_ddot = M_y \\ (Q_y - C_y*q_y_dot - D_y - G_y);"
"dx_p_dt = [ x_k_dot; theta_y_dot; q_y_ddot(1); q_y_ddot(2) ];"
"end"
];
write_block_file('pitch_dynamics.m', pitchCode);

% --- yaw_dynamics.m
yawCode = [
"function dx_z_dt = yaw_dynamics(t, x_z, tau_z)"
"%% yaw_dynamics 1-DOF X-Y plane (Simulink Data Store friendly)"
"% x_z = [theta_z ; theta_z_dot]"
""
"I_k_param = I_k_param; I_z = I_z; I_w = I_w; r_k = r_k; r_w = r_w; alpha_rad = alpha_rad;"
""
"theta_z = x_z(1); theta_z_dot = x_z(2);"
""
"num = I_k_param * (r_k / r_w) * tau_z;"
"den = I_k_param * I_z + 3 * (I_k_param + I_z) * I_w * (r_k / r_w)^2 * sin(alpha_rad)^2;"
"theta_z_ddot = num / den;"
""
"dx_z_dt = [ theta_z_dot; theta_z_ddot ];"
"end"
];
write_block_file('yaw_dynamics.m', yawCode);

fprintf('Dynamics .m files written.\n');

%% 2. Create Simulink model and blocks
fprintf('Creating Simulink model "%s"...\n', modelName);
new_system(modelName);
open_system(modelName);

% layout settings
x0 = 30; dx = 200; y0 = 30; dy = 140;

% Create Data Store Memory blocks with initial values
paramList = {
    'm_a',    68;
    'm_k',    19.6;
    'l',      0.38;
    'I_x',    12.1;
    'I_y',    11.67;
    'I_z',    1.08;
    'I_w',    0.26;
    'I_k_param', 0.38;
    'r_k',    0.22;
    'r_w',    0.1;
    'alpha_rad', deg2rad(65.5);
    'g',      9.81
    };

for i=1:size(paramList,1)
    name = paramList{i,1};
    val  = paramList{i,2};
    blockPath = sprintf('%s/%s', modelName, name);
    add_block('simulink/Signal Routing/Data Store Memory', blockPath, ...
        'Position',[x0, y0 + (i-1)*30, x0+120, y0+ (i-1)*30 +22]);
    % set initial value
    set_param(blockPath, 'Header','', 'InitialValue', sprintf('%g', val));
end

%% Create Inports for torques and Outports for states
add_block('simulink/Sources/In1', [modelName '/tau_x'], 'Position',[350, 40, 380, 60]);
add_block('simulink/Sources/In1', [modelName '/tau_y'], 'Position',[350, 180, 380, 200]);
add_block('simulink/Sources/In1', [modelName '/tau_z'], 'Position',[350, 320, 380, 340]);

% Create Subsystems (MATLAB Function blocks) for roll, pitch, yaw
rollBlk = [modelName '/RollDynamics'];
pitchBlk = [modelName '/PitchDynamics'];
yawBlk = [modelName '/YawDynamics'];

add_block('simulink/User-Defined Functions/MATLAB Function', rollBlk, ...
    'Position',[520, 30, 760, 140]);
add_block('simulink/User-Defined Functions/MATLAB Function', pitchBlk, ...
    'Position',[520, 170, 760, 280]);
add_block('simulink/User-Defined Functions/MATLAB Function', yawBlk, ...
    'Position',[520, 310, 760, 380]);

% Set the MATLAB Function block code to call the .m functions
% For roll: input x (4x1), tau_x -> output dx (4x1)
set_param(rollBlk, 'FunctionName', 'fcn');
rollCodeBlock = [...
    'function dx = fcn(x, tau_x)\n', ...
    ' %#ok<INUSD>\n', ...
    ' dx = roll_dynamics(0, x, tau_x);\n', ...
    'end'];
set_param(rollBlk, 'Script', rollCodeBlock);

% For pitch
set_param(pitchBlk, 'FunctionName', 'fcn');
pitchCodeBlock = [...
    'function dx = fcn(x, tau_y)\n', ...
    ' %#ok<INUSD>\n', ...
    ' dx = pitch_dynamics(0, x, tau_y);\n', ...
    'end'];
set_param(pitchBlk, 'Script', pitchCodeBlock);

% For yaw
set_param(yawBlk, 'FunctionName', 'fcn');
yawCodeBlock = [...
    'function dx = fcn(x, tau_z)\n', ...
    ' %#ok<INUSD>\n', ...
    ' dx = yaw_dynamics(0, x, tau_z);\n', ...
    'end'];
set_param(yawBlk, 'Script', yawCodeBlock);

%% Add Inport blocks for states (to allow external initial states / integration)
add_block('simulink/Sources/In1', [modelName '/x_r_in'], 'Position',[110, 40, 140, 80]);
add_block('simulink/Sources/In1', [modelName '/x_p_in'], 'Position',[110, 180, 140, 220]);
add_block('simulink/Sources/In1', [modelName '/x_z_in'], 'Position',[110, 320, 140, 360]);

% Add Integrator blocks (to integrate state derivatives)
add_block('simulink/Continuous/Integrator', [modelName '/int_roll'], 'Position',[820, 30, 910, 110], 'InitialCondition','[0;0;0;0]');
add_block('simulink/Continuous/Integrator', [modelName '/int_pitch'], 'Position',[820, 170, 910, 250], 'InitialCondition','[0;0;0;0]');
add_block('simulink/Continuous/Integrator', [modelName '/int_yaw'], 'Position',[820, 310, 910, 370], 'InitialCondition','[0;0]');

% Add Outports for the integrated states
add_block('simulink/Sinks/Out1', [modelName '/roll_out'], 'Position',[980, 40, 1010, 60]);
add_block('simulink/Sinks/Out1', [modelName '/pitch_out'], 'Position',[980, 180, 1010, 200]);
add_block('simulink/Sinks/Out1', [modelName '/yaw_out'], 'Position',[980, 320, 1010, 340]);

%% Connect tau inputs to dynamics blocks
add_line(modelName, 'tau_x/1', 'RollDynamics/2', 'autorouting','on');
add_line(modelName, 'tau_y/1', 'PitchDynamics/2', 'autorouting','on');
add_line(modelName, 'tau_z/1', 'YawDynamics/2', 'autorouting','on');

%% Connect state inputs to dynamics blocks
add_line(modelName, 'x_r_in/1', 'RollDynamics/1', 'autorouting','on');
add_line(modelName, 'x_p_in/1', 'PitchDynamics/1', 'autorouting','on');
add_line(modelName, 'x_z_in/1', 'YawDynamics/1', 'autorouting','on');

%% Connect dynamics outputs to integrators
add_line(modelName, 'RollDynamics/1', 'int_roll/1', 'autorouting','on');
add_line(modelName, 'PitchDynamics/1', 'int_pitch/1', 'autorouting','on');
add_line(modelName, 'YawDynamics/1', 'int_yaw/1', 'autorouting','on');

%% Connect integrator outputs to Outports
add_line(modelName, 'int_roll/1', 'roll_out/1', 'autorouting','on');
add_line(modelName, 'int_pitch/1', 'pitch_out/1', 'autorouting','on');
add_line(modelName, 'int_yaw/1', 'yaw_out/1', 'autorouting','on');

%% (Optional) connect x_in to integrator initial conditions via Sum or Manual Switch
% For simplicity, we leave integrators initial conditions set in block dialog.

%% Save and close system
fprintf('Saving model as %s.slx ...\n', modelName);
save_system(modelName, [modelName '.slx']);
close_system(modelName);

fprintf('Done. Model saved as %s.slx in current folder.\n', [modelName '.slx']);
fprintf('Open with: open_system(''%s.slx'')\n', modelName);
end

%% Helper to write .m files
function write_block_file(fname, lines)
fid = fopen(fname, 'w');
if fid < 0
    error('Unable to create %s in current folder.', fname);
end
for k = 1:numel(lines)
    fprintf(fid, '%s\n', lines{k});
end
fclose(fid);
end
