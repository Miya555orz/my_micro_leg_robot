function result = single_link_wheel_leg_lqr()
%SINGLE_LINK_WHEEL_LEG_LQR Calculate LQR gains for the single-link wheel-leg robot.
%
% Author: Miya Zheng
% Created: 2026-07-30
%
% How to use:
%   result = single_link_wheel_leg_lqr();
%   Check result.KwheelTorque and result.KfirmwareCurrent.
%   Tune Q/R in single_link_wheel_leg_params.m, then rerun.

p = single_link_wheel_leg_params();
model = single_link_wheel_leg_linear_model(p);

Ktorque = lqr(model.A, model.Bw, p.Q, p.R);

% Firmware uses two independent wheel current commands. Split total wheel torque
% equally, then convert torque to q-axis current. The generated 2x6 matrix maps
% to firmware state order:
%   [pitch, pitch_rate, left_pos, left_speed, right_pos, right_speed]
perWheelTorqueToCurrent = 1 / (2 * p.motorTorqueConstantNmPerA * p.motorGearRatio * p.motorEfficiency);
Kcurrent4 = Ktorque * perWheelTorqueToCurrent;
Kfirmware = zeros(2, 6);
Kfirmware(1, [1, 2]) = Kcurrent4([1, 2]);
Kfirmware(2, [1, 2]) = Kcurrent4([1, 2]);

% The reduced model uses wheel ground displacement s (m). If firmware reads
% wheel encoder angle/speed (rad, rad/s), convert feedback gains by s = R * phi.
wheelStateScale = 1.0;
if isfield(p, 'firmwareWheelStateUseMeter') && ~p.firmwareWheelStateUseMeter
    wheelStateScale = p.wheelRadiusM;
end

% Average left/right wheel feedback into the reduced single-link displacement channel.
Kfirmware(1, 3) = 0.5 * Kcurrent4(3) * wheelStateScale;
Kfirmware(1, 5) = 0.5 * Kcurrent4(3) * wheelStateScale;
Kfirmware(2, 3) = 0.5 * Kcurrent4(3) * wheelStateScale;
Kfirmware(2, 5) = 0.5 * Kcurrent4(3) * wheelStateScale;
Kfirmware(1, 4) = 0.5 * Kcurrent4(4) * wheelStateScale;
Kfirmware(1, 6) = 0.5 * Kcurrent4(4) * wheelStateScale;
Kfirmware(2, 4) = 0.5 * Kcurrent4(4) * wheelStateScale;
Kfirmware(2, 6) = 0.5 * Kcurrent4(4) * wheelStateScale;

result.params = p;
result.model = model;
result.KwheelTorque = Ktorque;
result.KfirmwareCurrent = Kfirmware;
result.closedLoopEigen = eig(model.A - model.Bw * Ktorque);

fprintf('Single-link wheel-leg LQR ready.\n');
fprintf('K wheel torque [theta theta_dot s s_dot]:\n');
disp(Ktorque);
fprintf('Firmware K current 2x6:\n');
disp(Kfirmware);
fprintf('Closed-loop eigenvalues:\n');
disp(result.closedLoopEigen);

single_link_wheel_leg_export(result);
end
