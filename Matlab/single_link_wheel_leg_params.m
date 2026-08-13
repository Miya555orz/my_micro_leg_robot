function p = single_link_wheel_leg_params()
%SINGLE_LINK_WHEEL_LEG_PARAMS Parameters for the mini single-link wheel-leg model.
%
% Author: Miya Zheng
% Created: 2026-07-30
%
% How to use:
%   1. Fill the mass, COM, and inertia values from the CAD mass properties.
%   2. Run single_link_wheel_leg_lqr to calculate the reduced LQR gains.
%   3. Run single_link_wheel_leg_export to copy gains into mini_robot_config.h.
%
% Coordinate convention:
%   theta = body/link pitch angle from upright, forward positive (rad)
%   s     = wheel ground displacement, forward positive (m)
%   Tw    = total left+right wheel torque, forward positive (N*m)
%   Tp    = auxiliary joint/servo torque, forward-body positive (N*m)

p.name = 'Miya mini single-link wheel-leg';
p.author = 'Miya Zheng';
p.sourceStep = 'D:\github_prj\Micro-Wheeled_leg-Robot\1.RobotModel\OriginalRobotModel.stp';
p.stepLengthUnit = 'mm';

% Environment.
p.g = 9.80665;                 % Gravity acceleration (m/s^2)

% Wheel and drive train. Replace torqueConstantNmPerA with your MiyaFOC motor value.
p.wheelRadiusM = 0.030;        % Wheel radius (m)
p.wheelTrackM = 0.160;         % Left-right wheel distance (m)
p.wheelMassKg = 0.120;         % Total two-wheel equivalent rolling mass (kg)
p.wheelInertiaKgM2 = 0.5 * p.wheelMassKg * p.wheelRadiusM^2;
p.motorTorqueConstantNmPerA = 0.080; % One motor torque per q-axis current (N*m/A)
p.motorGearRatio = 1.0;        % Hub motor/direct-drive gear ratio
p.motorEfficiency = 0.85;      % Conservative torque transfer factor
p.maxWheelCurrentA = 2.5;      % First standing-test current limit per wheel (A)

% Firmware mapping. false means firmware wheel states are wheel angle/speed (rad, rad/s).
% true means firmware wheel states are ground displacement/speed (m, m/s).
p.firmwareWheelStateUseMeter = false;

% Equivalent single-link body. These are placeholders and must be measured from CAD.
p.bodyMassKg = 0.900;          % Body, battery, PCB, servo, and upper link equivalent mass (kg)
p.linkMassKg = 0.120;          % One-side links reflected into pitch model (kg)
p.equivalentMassKg = p.bodyMassKg + p.linkMassKg;
p.comHeightM = 0.095;          % Equivalent COM height above wheel axle at upright pose (m)
p.bodyInertiaPitchKgM2 = 0.0045; % Equivalent pitch inertia about COM (kg*m^2)

% Optional joint/servo channel. Current firmware mainly uses servo position, so Tp is exported separately.
p.servoTorqueLimitNm = 1.0;    % Auxiliary torque limit for simulation only (N*m)
p.servoPositionCenter = 2048;  % STS3032 center position tick
p.servoTickPerRad = 4096 / (2 * pi);
p.servoAssistGainTickPerRad = 120; % Pitch-to-servo bias for early tests (tick/rad)

% LQR weights. State x = [theta; theta_dot; s; s_dot]. Input u = total wheel torque Tw.
p.Q = diag([180, 6, 8, 1]);
p.R = 0.65;

% Simulation defaults.
p.initialState = [deg2rad(4); 0; 0; 0];
p.simTimeS = 3.0;
p.controlPeriodS = 0.001;
end

