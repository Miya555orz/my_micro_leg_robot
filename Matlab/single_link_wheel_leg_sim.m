function sim = single_link_wheel_leg_sim()
%SINGLE_LINK_WHEEL_LEG_SIM Simulate the reduced single-link wheel-leg LQR model.
%
% Author: Miya Zheng
% Created: 2026-07-30
%
% How to use:
%   sim = single_link_wheel_leg_sim();
%   Inspect pitch convergence and wheel displacement before copying gains.

result = single_link_wheel_leg_lqr();
p = result.params;
model = result.model;
K = result.KwheelTorque;

f = @(t, x) closed_loop_dynamics(t, x, model, K, p);
[t, x] = ode45(f, [0 p.simTimeS], p.initialState);

sim.t = t;
sim.x = x;
sim.pitchDeg = rad2deg(x(:, 1));
sim.wheelDisplacementM = x(:, 3);

figure('Name', 'Single-link wheel-leg LQR simulation');
subplot(2, 1, 1);
plot(t, sim.pitchDeg, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Body pitch response');

subplot(2, 1, 2);
plot(t, sim.wheelDisplacementM, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Wheel displacement (m)');
title('Wheel displacement');
end

function dx = closed_loop_dynamics(~, x, model, K, p)
Tw = -K * x;
Tw = max(min(Tw, 2 * p.maxWheelCurrentA * p.motorTorqueConstantNmPerA * p.motorGearRatio * p.motorEfficiency), ...
         -2 * p.maxWheelCurrentA * p.motorTorqueConstantNmPerA * p.motorGearRatio * p.motorEfficiency);
dx = model.A * x + model.Bw * Tw;
end

