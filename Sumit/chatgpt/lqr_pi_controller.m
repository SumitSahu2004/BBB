% lqr_pi_controller.m
% Implements LQR outer-loop and PI inner-loop, torque conversion to motors,
% and feedforward compensation as in the paper. :contentReference[oaicite:12]{index=12}

function [ux_motor, uy_motor, logs] = lqr_pi_controller(x_aug, params, gains, integrators, dt)
% Inputs:
%  - x_aug : 5x1 augmented state (pos, angle, v_pos, ang_vel, int_pos)
%  - params : parameter struct
%  - gains : struct with Kr, Kp, Kpi (inner PI gains)
%  - integrators : struct with current integral errors
%  - dt : timestep
%
% Outputs:
%  - ux_motor, uy_motor : torque commands applied as equivalent body torques
%  - logs: struct with useful intermediate outputs

    % Outer-loop: LQR provides speed reference (paper: vx_ref, vy_ref)
    % Here we use K as a row that multiplies x_aug to get reference velocity
    vx_ref = -gains.Kr * x_aug;   % roll-plane speed reference (scalar)
    vy_ref = -gains.Kp * x_aug;   % pitch-plane speed reference (scalar)

    % Inner-loop PI controller (velocity tracking) - paper Eq (28..36)
    % e = v_ref - measured_v  (here measured_v use x_aug(3) as v_pos state)
    % Note: mapping is approximate because our simplified linear model mixes variables.

    v_meas = x_aug(3);  % measured linear velocity approx
    w_meas = x_aug(4);  % measured angular velocity approx

    e_vx = vx_ref - v_meas;
    e_vy = vy_ref - w_meas;

    % integrate
    integrators.roll = integrators.roll + e_vx * dt;
    integrators.pitch = integrators.pitch + e_vy * dt;

    % PI control law (u = Kp*e + Ki*int + feedforward)
    ux_eq = gains.Kpi.Kpr * e_vx + gains.Kpi.Kir * integrators.roll + gains.uxf;
    uy_eq = gains.Kpi.Kpp * e_vy + gains.Kpi.Kip * integrators.pitch + gains.uyf;

    % Torque conversion eq (map [ux_eq; uy_eq] -> three motor torques)
    % Using the form from Eq. (20) (paper). We'll compute motor torques tau_i.
    alpha = params.alpha;
    % Build conversion matrix from virtual torques to motor torques (3x2)
    % (paper has a 3x3 with z-axis but we use z=0 for planar)
    M = [ cos(alpha)  sin(alpha);
          cos(alpha-2*pi/3) sin(alpha-2*pi/3);
          cos(alpha+2*pi/3) sin(alpha+2*pi/3) ];
    % Solve for motor contributions (least-squares) — map ux,uy to motor torques
    motor_tau = M * [ux_eq; uy_eq];

    ux_motor = motor_tau(1);
    uy_motor = motor_tau(2);  % note: this returns first two motor torques; third available if needed

    % Logs
    logs.vx_ref = vx_ref;
    logs.vy_ref = vy_ref;
    logs.e_vx = e_vx;
    logs.e_vy = e_vy;
    logs.integrators = integrators;
    logs.ux_eq = ux_eq;
    logs.uy_eq = uy_eq;
    logs.motor_tau = motor_tau;
end
