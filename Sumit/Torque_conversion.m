function J = kinematic_model()
% GLOBAL PARAMETERS
global rk rw a

% Jacobian for wheel → body rotation mapping
J = [ cos(a)/rw      cos(a)/rw       cos(a)/rw;
     -sin(a)/rw      sin(a)/rw              0 ;
    -(rk/rw)        -(rk/rw)         -(rk/rw) ];
end
