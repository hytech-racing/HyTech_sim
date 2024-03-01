clear;
close all;
clc;

%% Parameters
dt = 0.001;                                     % s

m = 250;                                        % kg, total vehicle mass
Iz_vehicle = 100;                               % kgm^2
g = 9.81;                                       % m/s^2
rho = 1.293;                                    % kg/m^3, density of air

cD = 1.23;                                      % Coeff. of drag
cL = 3.71;                                      % Coeff. of lift
frontalA = 1;                                   % m^2, frontal area

wb = 1.535;                                     % m, wheelbase
lr = 0.78;                                      % m, CG to rear axle
lf = wb - lr;                                   % m, CG to front axle
hCG = 0.2;                                      % m, CG height
TLLTD = 0.5;                                    % Total lateral load transfer distribution, front

track = 1.2;                                    % m, total track width
tf = track / 2;                                 % m, front half track
tr = track / 2;                                 % m, rear half track

GR = 11.86;                                     % Gear reduction
Ix_motor = 0.000274;                            % kgm^2, motor mass moment of inertia
Ix_wheel = (0.07829 + Ix_motor*GR*GR);          % kgm^2, wheel + motor mass moment of inertia, wheel mass = 4.6 kg
b = 0.09;                                        % N.m.s/rad, motor damping factor
gearboxEff = 1;                               % Gearbox efficiency
motorOmegaLimit = 2094;                         % rad/s, motor mech. speed limit

R_wheel = 0.2;                                  % m, tire radius

c_roll = 0.1;                                   % Rolling resistance coefficient

R_FL = sqrt(tf^2 + lf^2);                       % m, distance from CG to FL wheel center
R_FR = R_FL;                                    % m
R_RL = sqrt(tr^2 + lr^2);                       % m
R_RR = R_RL;                                    % m

e_FL = atan(tf/lf);                             % rad, perpendicular vector to the direction vector 
                                                %      from CG to FL wheel center, angle between this
                                                %      perpendicular vector with vehicle y-axis

e_FR = atan(lf/tf);                             % rad, perpendicular vector to the direction vector 
                                                %      from CG to FL wheel center, angle between this
                                                %      perpendicular vector with vehicle x-axis

e_RL = atan(lr/tr);                             % rad, perpendicular vector to the direction vector 
                                                %      from CG to FL wheel center, angle between this
                                                %      perpendicular vector with vehicle x-axis

e_RR = atan(tr/lr);                             % rad, perpendicular vector to the direction vector 
                                                %      from CG to FL wheel center, angle between this
                                                %      perpendicular vector with vehicle y-axis

% wheelSS.A = -b/Ix_wheel;
% wheelSS.B = [1/Ix_wheel -R_wheel/Ix_wheel];
% wheelSS.C = 1;
% wheelSS.D = [0 0];
% 
% omegaFL = 0;

A = load('PSS LATERAL FORCE COEFFS.mat').A;
B = load('PSS ALIGNING TORQUE COEFFS.mat').B;
D = load('PLS LONGITUDINAL FORCE COEFFS.mat').D;

E = load('COMBINED LONGITUDINAL FORCE COEFFS.mat').E;
F = load('COMBINED LATERAL FORCE COEFFS.mat').F;
G = load('COMBINED ALIGNING TORQUE COEFFS.mat').G;
C = load('COMBINED OVERTURNING MOMENT COEFFS.mat').C;

tireFactor_X_Accel = 0.6;
tireFactor_X_Brake = 0.6;
tireFactor_Y = 0.6;

%%
% load data0053.mat % AutoX
load data0058.mat % Endurance

motorTorqueFLTime = data.MOTOR_CONTROLLER.mc_fl.feedback_torque(:,1);
motorTorqueFRTime = data.MOTOR_CONTROLLER.mc_fr.feedback_torque(:,1);
motorTorqueRLTime = data.MOTOR_CONTROLLER.mc_rl.feedback_torque(:,1);
motorTorqueRRTime = data.MOTOR_CONTROLLER.mc_rr.feedback_torque(:,1);

motorTorqueFL = data.MOTOR_CONTROLLER.mc_fl.feedback_torque(:,2);
motorTorqueFR = data.MOTOR_CONTROLLER.mc_fr.feedback_torque(:,2);
motorTorqueRL = data.MOTOR_CONTROLLER.mc_rl.feedback_torque(:,2);
motorTorqueRR = data.MOTOR_CONTROLLER.mc_rr.feedback_torque(:,2);

wheelSpeedDataFL = data.MOTOR_CONTROLLER.mc_fl.speed(:,2).*0.2.*0.1047198./11.86;
wheelSpeedDataFR = data.MOTOR_CONTROLLER.mc_fr.speed(:,2).*0.2.*0.1047198./11.86;
wheelSpeedDataRL = data.MOTOR_CONTROLLER.mc_rl.speed(:,2).*0.2.*0.1047198./11.86;
wheelSpeedDataRR = data.MOTOR_CONTROLLER.mc_rr.speed(:,2).*0.2.*0.1047198./11.86;

wheelSpeedDataFLTime = data.MOTOR_CONTROLLER.mc_fl.speed(:,1);
wheelSpeedDataFRTime = data.MOTOR_CONTROLLER.mc_fr.speed(:,1);
wheelSpeedDataRLTime = data.MOTOR_CONTROLLER.mc_rl.speed(:,1);
wheelSpeedDataRRTime = data.MOTOR_CONTROLLER.mc_rr.speed(:,1);

[uniqueTimeFL, indFL] = unique(motorTorqueFLTime, 'stable');
[uniqueTimeFR, indFR] = unique(motorTorqueFRTime, 'stable');
[uniqueTimeRL, indRL] = unique(motorTorqueRLTime, 'stable');
[uniqueTimeRR, indRR] = unique(motorTorqueRRTime, 'stable');

timelsim = uniqueTimeFL(1):dt:uniqueTimeFL(end);

motorTorqueFLInterp = interp1(uniqueTimeFL, motorTorqueFL(indFL), timelsim)';
motorTorqueFRInterp = interp1(uniqueTimeFR, motorTorqueFR(indFR), timelsim)';
motorTorqueRLInterp = interp1(uniqueTimeRL, motorTorqueRL(indRL), timelsim)';
motorTorqueRRInterp = interp1(uniqueTimeRR, motorTorqueRR(indRR), timelsim)';

motorTorqueFLInterp(isnan(motorTorqueFLInterp)) = 0;
motorTorqueFRInterp(isnan(motorTorqueFRInterp)) = 0;
motorTorqueRLInterp(isnan(motorTorqueRLInterp)) = 0;
motorTorqueRRInterp(isnan(motorTorqueRRInterp)) = 0;

motorTorqueFLInterp(abs(motorTorqueFLInterp) < 1) = 0;
motorTorqueFRInterp(abs(motorTorqueFRInterp) < 1) = 0;
motorTorqueRLInterp(abs(motorTorqueRLInterp) < 1) = 0;
motorTorqueRRInterp(abs(motorTorqueRRInterp) < 1) = 0;

motorTorqueFLInterp(abs(motorTorqueFLInterp) > 22) = 22;
motorTorqueFRInterp(abs(motorTorqueFRInterp) > 22) = 22;
motorTorqueRLInterp(abs(motorTorqueRLInterp) > 22) = 22;
motorTorqueRRInterp(abs(motorTorqueRRInterp) > 22) = 22;

[uniqueTimeFL, indFL] = unique(wheelSpeedDataFLTime, 'stable');
[uniqueTimeFR, indFR] = unique(wheelSpeedDataFRTime, 'stable');
[uniqueTimeRL, indRL] = unique(wheelSpeedDataRLTime, 'stable');
[uniqueTimeRR, indRR] = unique(wheelSpeedDataRRTime, 'stable');

wheelSpeedDataFLInterp = interp1(uniqueTimeFL, wheelSpeedDataFL(indFL), timelsim)';
wheelSpeedDataFRInterp = interp1(uniqueTimeFR, wheelSpeedDataFR(indFR), timelsim)';
wheelSpeedDataRLInterp = interp1(uniqueTimeRL, wheelSpeedDataRL(indRL), timelsim)';
wheelSpeedDataRRInterp = interp1(uniqueTimeRR, wheelSpeedDataRR(indRR), timelsim)';


timelsim = timelsim';
maxval = round(timelsim(end));

figure
hold on
plot(motorTorqueFLInterp)
plot(motorTorqueFRInterp)
plot(motorTorqueRLInterp)
plot(motorTorqueRRInterp)

%%
[commonTime, iA, iB] = intersect(out.wheelLinearSpeed.Time(:, 1), timelsim);

simWheelSpeedFL = out.wheelLinearSpeed.Data(:, 1);
simWheelSpeedFL = simWheelSpeedFL(iA);

simWheelSpeedFR = out.wheelLinearSpeed.Data(:, 2);
simWheelSpeedFR = simWheelSpeedFR(iA);

simWheelSpeedRL = out.wheelLinearSpeed.Data(:, 3);
simWheelSpeedRL = simWheelSpeedRL(iA);

simWheelSpeedRR = out.wheelLinearSpeed.Data(:, 4);
simWheelSpeedRR = simWheelSpeedRR(iA);

wheelSpeedDataFLInterp = wheelSpeedDataFLInterp(iB);
wheelSpeedDataFRInterp = wheelSpeedDataFRInterp(iB);
wheelSpeedDataRLInterp = wheelSpeedDataRLInterp(iB);
wheelSpeedDataRRInterp = wheelSpeedDataRRInterp(iB);

percentDiffFL = (simWheelSpeedFL - wheelSpeedDataFLInterp) ./ wheelSpeedDataFLInterp .* 100;
percentDiffFR = (simWheelSpeedFR - wheelSpeedDataFRInterp) ./ wheelSpeedDataFRInterp .* 100;
percentDiffRL = (simWheelSpeedRL - wheelSpeedDataRLInterp) ./ wheelSpeedDataRLInterp .* 100;
percentDiffRR = (simWheelSpeedRR - wheelSpeedDataRRInterp) ./ wheelSpeedDataRRInterp .* 100;

percentDiffFL(abs(percentDiffFL) > 40) = 40 * sign(percentDiffFL(abs(percentDiffFL) > 40));
percentDiffFR(abs(percentDiffFR) > 40) = 40 * sign(percentDiffFR(abs(percentDiffFR) > 40));
percentDiffRL(abs(percentDiffRL) > 40) = 40 * sign(percentDiffRL(abs(percentDiffRL) > 40));
percentDiffRR(abs(percentDiffRR) > 40) = 40 * sign(percentDiffRR(abs(percentDiffRR) > 40));

percentDiffFL(isnan(percentDiffFL)) = 0;
percentDiffFR(isnan(percentDiffFR)) = 0;
percentDiffRL(isnan(percentDiffRL)) = 0;
percentDiffRR(isnan(percentDiffRR)) = 0;


%%
% close all
figure

tiledlayout(4, 2)
ax1 = nexttile;
hold on
grid on 
plot(timelsim, motorTorqueFLInterp)
plot(commonTime, wheelSpeedDataFLInterp, '--') 
plot(commonTime, simWheelSpeedFL)
legend('Input Torque', 'Test Data Speed', 'Sim Speed')
title('FL')

ax2 = nexttile;
hold on
grid on 
plot(timelsim, motorTorqueFRInterp)
plot(commonTime, wheelSpeedDataFRInterp, '--') 
plot(commonTime, simWheelSpeedFR)
legend('Input Torque', 'Test Data Speed', 'Sim Speed')
title('FR')

ax3 = nexttile;
hold on
grid on 
plot(commonTime, percentDiffFL)
txt = sprintf('FL Percent Diff Avg: %.2f', mean(percentDiffFL));
annotation('textbox',...
    [0.0855 0.6739 0.0531 0.0225],...
    'String',txt,...
    'FitBoxToText','on');
title('FL % Diff')

ax4 = nexttile;
hold on
grid on 
plot(commonTime, percentDiffFR)
txt = sprintf('FR Percent Diff Avg: %.2f', mean(percentDiffFR));
annotation('textbox',...
    [0.5551 0.6722 0.0681 0.0234],...
    'String',txt,...
    'FitBoxToText','on');
title('FR % Diff')

ax5 = nexttile;
hold on
grid on 
plot(timelsim, motorTorqueRLInterp)
plot(commonTime, wheelSpeedDataRLInterp, '--') 
plot(commonTime, simWheelSpeedRL)
legend('Input Torque', 'Test Data Speed', 'Sim Speed')
title('RL')

ax6 = nexttile;
hold on
grid on 
plot(timelsim, motorTorqueRRInterp)
plot(commonTime, wheelSpeedDataRRInterp, '--') 
plot(commonTime, simWheelSpeedRR)
legend('Input Torque', 'Test Data Speed', 'Sim Speed')
title('RR')

ax7 = nexttile;
hold on
grid on 
plot(commonTime, percentDiffRL)
txt = sprintf('RL Percent Diff Avg: %.2f', mean(percentDiffRL));
annotation('textbox',...
    [0.0893 0.2316 0.0565 0.02508],...
    'String',txt,...
    'FitBoxToText','on');
title('RL % Diff')

ax8 = nexttile;
hold on
grid on 
plot(commonTime, percentDiffRR)
txt = sprintf('RR Percent Diff Avg: %.2f', mean(percentDiffRR));
annotation('textbox',...
    [0.5535 0.2307 0.0719 0.0234],...
    'String',txt,...
    'FitBoxToText','on');
title('RR % Diff')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'x')
