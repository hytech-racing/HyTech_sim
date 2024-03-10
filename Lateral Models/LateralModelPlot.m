
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
