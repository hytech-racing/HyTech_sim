%%
% load data0053.mat % AutoX 26s - 70s
load data0058.mat % Endurance
% load data0234.mat % Left Hand Continuous Corner 78.631s - 110s

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

steeringTime = data.MCU.analog.steering_2(:,1);
steeringData = -0.111*data.MCU.analog.steering_2(:,2) + 260;

steeringData(abs(steeringData) < 1) = 0;


[uniqueTimeFL, indFL] = unique(motorTorqueFLTime, 'stable');
[uniqueTimeFR, indFR] = unique(motorTorqueFRTime, 'stable');
[uniqueTimeRL, indRL] = unique(motorTorqueRLTime, 'stable');
[uniqueTimeRR, indRR] = unique(motorTorqueRRTime, 'stable');

[uniqueTimeSteer, indSteer] = unique(steeringTime, 'stable');

timelsim = uniqueTimeFL(1):dt:uniqueTimeFL(end);

motorTorqueFLInterp = interp1(uniqueTimeFL, motorTorqueFL(indFL), timelsim)';
motorTorqueFRInterp = interp1(uniqueTimeFR, motorTorqueFR(indFR), timelsim)';
motorTorqueRLInterp = interp1(uniqueTimeRL, motorTorqueRL(indRL), timelsim)';
motorTorqueRRInterp = interp1(uniqueTimeRR, motorTorqueRR(indRR), timelsim)';

steeringDataInterp = interp1(uniqueTimeSteer, steeringData(indSteer), timelsim)';

motorTorqueFLInterp(isnan(motorTorqueFLInterp)) = 0;
motorTorqueFRInterp(isnan(motorTorqueFRInterp)) = 0;
motorTorqueRLInterp(isnan(motorTorqueRLInterp)) = 0;
motorTorqueRRInterp(isnan(motorTorqueRRInterp)) = 0;

steeringDataInterp(isnan(steeringDataInterp)) = 0;

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

% [motorTorqueFRInterp, motorTorqueFLInterp] = alignsignals(motorTorqueFRInterp, motorTorqueFLInterp);
% [motorTorqueRLInterp, motorTorqueFLInterp] = alignsignals(motorTorqueRLInterp, motorTorqueFLInterp);
% [motorTorqueRRInterp, motorTorqueFLInterp] = alignsignals(motorTorqueRRInterp, motorTorqueFLInterp);
motorTorqueFLInitInd = find((motorTorqueFLInterp > 0), 1, 'first');
motorTorqueFRInitInd = find((motorTorqueFRInterp > 0), 1, 'first');
motorTorqueRLInitInd = find((motorTorqueRLInterp > 0), 1, 'first');
motorTorqueRRInitInd = find((motorTorqueRRInterp > 0), 1, 'first');

latestInitInd = max([motorTorqueFLInitInd motorTorqueFRInitInd motorTorqueRLInitInd motorTorqueRRInitInd]);
motorTorqueFLInterp(1:latestInitInd) = 0;
motorTorqueFRInterp(1:latestInitInd) = 0;
motorTorqueRLInterp(1:latestInitInd) = 0;
motorTorqueRRInterp(1:latestInitInd) = 0;

% For HT07 data, negative steer = turning right, change to SAE standard
steeringDataMap = [-130 130];
wheelSteerRange = [23 -23];
steeringDataInput = interp1(steeringDataMap, wheelSteerRange, steeringDataInterp);


% wheelSteerDataInterpMagnitude = interp1(abs(driverSteerKin), abs(wheelSteerKin), abs(steeringDataInterp));
% wheelSteerDataInterpDir = sign(steeringDataInterp);

% wheelSteerDataInterp = wheelSteerDataInterpMagnitude .* wheelSteerDataInterpDir;


% wheelSteerDataInterp = wheelSteerDataInterp .* -1;

% wheelSteerDataInterp = smoothdata(wheelSteerDataInterp, 'movmedian');


timelsim = timelsim';

% maxval = round(timelsim(end));
% 
figure
hold on
plot(timelsim, motorTorqueFLInterp)
plot(timelsim, motorTorqueFRInterp)
plot(timelsim, motorTorqueRLInterp)
plot(timelsim, motorTorqueRRInterp)

figure
hold on
plot(timelsim, steeringDataInput)
legend('Driver Steer (Deg)')
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

figure
hold on
grid on
plot(out.bFrame.Speed)

% figure
% hold on
% grid on 
% plot(timelsim, motorTorqueFLInterp)
% plot(commonTime, wheelSpeedDataFLInterp, '--') 
% plot(commonTime, simWheelSpeedFL)
% legend('Input Torque [N-m]', 'Test Data Speed [m/s]', 'Sim Speed [m/s]')
% title('FL Wheel Linear Speed')
%%
figure
hold on
grid on
plot(out.bFrame.Yaw_Rate)
plot(out.bFrame.Vy_B)
legend('Yaw Rate [rad/s]', 'Vy_B [m/s]')

figure
hold on
grid on
plot(out.bFrame.Yaw_Accel)
%%
figure
hold on
axis equal
grid on
plot(out.positionVec.Y_POS_INERTIAL.Data, out.positionVec.X_POS_INERTIAL.Data)
plot(out.positionVec.Y_POS_INERTIAL.Data(1), out.positionVec.X_POS_INERTIAL.Data(1), '^', 'LineWidth', 3)
xlabel('Y (m)')
ylabel('X (m)')
legend('Inertial Position', 'Starting Point')
