function [A,B] = ball_linear_model(params)
% ball_linear_model (controllable canonical surrogate)
% Returns a guaranteed-controllable 5x5 A and 5x1 B in controllable canonical form.
% Use this when the symbolic Euler-Lagrange linearization is not yet available.
%
% State ordering (canonical): x = [x1; x2; x3; x4; x5]
% Input: single scalar u (equivalent torque)
%
% This surrogate is stable/controllable and suitable for LQR design and controller
% development. Replace with the paper's exact linearization for final validation.

    n = 5;

    % --- Desired pole locations for the surrogate dynamics ---
    % Choose stable real poles (tune as you like)
    desired_poles = [-1.0, -1.4, -1.8, -2.2, -2.6];  % moderately damped slow->faster

    % Build characteristic polynomial from desired poles
    poly_coefs = real(poly(desired_poles)); % returns [1, a1, a2, ..., an]

    % Controllable canonical (companion) matrix A_c and B_c
    % Companion A: last row = -a_n ... -a_1 (excluding leading 1), subdiagonal ones
    A = zeros(n);
    % Fill subdiagonal ones
    for i = 2:n
        A(i-1,i) = 1;   % note: this arrangement places states x1 .. xn in canonical order
    end
    % Place companion coefficients in the last row (negated)
    % poly_coefs = [1, a1, a2, ..., an] so
    a_coeffs = poly_coefs(2:end); % a1..an
    A(n,:) = -a_coeffs;

    % Single-input B in canonical form (only last element non-zero)
    B = zeros(n,1);
    B(n) = 1;

    % Quick controllability check and optional debug info
    Co = ctrb(A,B);
    rk = rank(Co);
    if rk < n
        warning('Surrogate canonical model is unexpectedly not controllable (rank=%d).', rk);
    else
        % fprintf('Surrogate canonical model is controllable (rank=%d).\n', rk);
    end

    % NOTE: This canonical form produces states that are not the same physical
    % states as the paper's variables. This is intentional: it's a controllable
    % surrogate for LQR/prototyping. Replace with exact A,B from symbolic
    % linearization when available for physical fidelity.
end
