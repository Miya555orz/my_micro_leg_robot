function model = single_link_wheel_leg_linear_model(p)
%SINGLE_LINK_WHEEL_LEG_LINEAR_MODEL Linear model for a single-link wheel-leg robot.
%
% Author: Miya Zheng
% Created: 2026-07-30
%
% How to use:
%   p = single_link_wheel_leg_params();
%   model = single_link_wheel_leg_linear_model(p);
%   K = lqr(model.A, model.Bw, p.Q, p.R);
%
% Model idea:
%   This is a reduced single-link wheel inverted-pendulum model, not a five-link
%   VMC model. The wheel torque both accelerates the ground contact and applies
%   an opposite reaction torque to the body/link pitch channel.

m = p.equivalentMassKg;
Mw = p.wheelMassKg;
h = p.comHeightM;
I = p.bodyInertiaPitchKgM2;
R = p.wheelRadiusM;
g = p.g;

M11 = Mw + m + p.wheelInertiaKgM2 / (R * R);
M12 = m * h;
M22 = I + m * h * h;
D = [M11, M12; M12, M22];

% Linearized acceleration equation around upright theta = 0:
%   D * [s_ddot; theta_ddot] = [Tw/R; m*g*h*theta - Tw + Tp]
Gtheta = [0; m * g * h];
Bw_acc = [1 / R; -1];
Bp_acc = [0; 1];
accTheta = D \ Gtheta;
accWheel = D \ Bw_acc;
accServo = D \ Bp_acc;

A = zeros(4, 4);
A(1, 2) = 1;
A(2, 1) = accTheta(2);
A(3, 4) = 1;
A(4, 1) = accTheta(1);

Bw = zeros(4, 1);
Bw(2) = accWheel(2);
Bw(4) = accWheel(1);

Bp = zeros(4, 1);
Bp(2) = accServo(2);
Bp(4) = accServo(1);

model.A = A;
model.Bw = Bw;
model.Bp = Bp;
model.B = [Bw, Bp];
model.C = eye(4);
model.D = zeros(4, 2);
model.massMatrix = D;
model.stateName = {'theta_rad', 'theta_rate_rps', 'wheel_s_m', 'wheel_speed_mps'};
model.inputName = {'wheel_total_torque_Nm', 'servo_aux_torque_Nm'};
end
