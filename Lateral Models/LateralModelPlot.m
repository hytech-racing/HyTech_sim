
%%
[commonTime, iA, iB] = intersect(out.wheelLinearSpeed.Time(:, 1), timelsim);

simWheelSpeedFL = out.wheelLinearSpeed.Data(:, 1);
simWheelSpeedFL = (simWheelSpeedFL(iA));

simWheelSpeedFR = out.wheelLinearSpeed.Data(:, 2);
simWheelSpeedFR = (simWheelSpeedFR(iA));

simWheelSpeedRL = out.wheelLinearSpeed.Data(:, 3);
simWheelSpeedRL = (simWheelSpeedRL(iA));

simWheelSpeedRR = out.wheelLinearSpeed.Data(:, 4);
simWheelSpeedRR = (simWheelSpeedRR(iA));

wheelSpeedDataFLInterpTrunc = wheelSpeedDataFLInterp(iB);
wheelSpeedDataFRInterpTrunc = wheelSpeedDataFRInterp(iB);
wheelSpeedDataRLInterpTrunc = wheelSpeedDataRLInterp(iB);
wheelSpeedDataRRInterpTrunc = wheelSpeedDataRRInterp(iB);

percentDiffFL = (simWheelSpeedFL - wheelSpeedDataFLInterpTrunc) ./ wheelSpeedDataFLInterpTrunc .* 100;
percentDiffFR = (simWheelSpeedFR - wheelSpeedDataFRInterpTrunc) ./ wheelSpeedDataFRInterpTrunc .* 100;
percentDiffRL = (simWheelSpeedRL - wheelSpeedDataRLInterpTrunc) ./ wheelSpeedDataRLInterpTrunc .* 100;
percentDiffRR = (simWheelSpeedRR - wheelSpeedDataRRInterpTrunc) ./ wheelSpeedDataRRInterpTrunc .* 100;

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
plot(commonTime, wheelSpeedDataFLInterpTrunc, '--') 
plot(commonTime, simWheelSpeedFL)
legend('Input Torque', 'Test Data Speed', 'Sim Speed')
title('FL')

ax2 = nexttile;
hold on
grid on 
plot(timelsim, motorTorqueFRInterp)
plot(commonTime, wheelSpeedDataFRInterpTrunc, '--') 
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
plot(commonTime, wheelSpeedDataRLInterpTrunc, '--') 
plot(commonTime, simWheelSpeedRL)
legend('Input Torque', 'Test Data Speed', 'Sim Speed')
title('RL')

ax6 = nexttile;
hold on
grid on 
plot(timelsim, motorTorqueRRInterp)
plot(commonTime, wheelSpeedDataRRInterpTrunc, '--') 
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
legend('Speed Expressed in B Frame')
% plot(out.bFrame.Vx_B)

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

% figure
% hold on
% grid on
% plot(out.bFrame.Yaw_Accel)
%% INERTIAL MAP
figure
hold on
axis equal
grid on
plot(out.iFrame.Y_POS_INERTIAL.Data, out.iFrame.X_POS_INERTIAL.Data)
plot(out.iFrame.Y_POS_INERTIAL.Data(1), out.iFrame.X_POS_INERTIAL.Data(1), '^', 'LineWidth', 3)
xlabel('Y (m)')
ylabel('X (m)')
legend('Inertial Position', 'Starting Point')

%% INERTIAL COMPASS

% Define unit vector length
cgVelL = 1;
bodyVelL = cgVelL;

CGheadingAngleRad = atan2(out.iFrame.Y_VEL_INERTIAL.Data, out.iFrame.X_VEL_INERTIAL.Data);

CGheadingAngleRad(isnan(CGheadingAngleRad)) = 0;
CGheadingAngleRad(CGheadingAngleRad < 0) = CGheadingAngleRad(CGheadingAngleRad < 0) + 2*pi;

cgVelHeadX = cos(CGheadingAngleRad) .* cgVelL;
cgVelHeadY = sin(CGheadingAngleRad) .* cgVelL;

bodyHeadingAngleRad = deg2rad(out.iFrame.YAW_ANGLE_DEG.Data);

bodyHeadingX = cos(bodyHeadingAngleRad) .* bodyVelL;
bodyHeadingY = sin(bodyHeadingAngleRad) .* bodyVelL;

figure
hold on
box on
grid on
plot(out.tout, rad2deg(CGheadingAngleRad), 'r')
plot(out.tout, rad2deg(bodyHeadingAngleRad), 'g')
legend('CG Vel. Dir.', 'Body Heading Dir.')

figure
hold on
box on
grid on
axis equal
cgVelLine = line([0 cgVelHeadY(1)], [0 cgVelHeadX(1)],'color' ,'r');
bodyHeadingLine = line([0 bodyHeadingY(1)], [0 bodyHeadingX(1)],'color' ,'g');

an = annotation('textbox', [.2, .8, .1, .1], 'String', 't = ');

xlim([-.7 .7])
ylim([-.7 .7])

for i = 1:20:length(cgVelHeadX)

    
    set(cgVelLine, 'YData', [0 cgVelHeadX(i)])
    set(cgVelLine, 'XData', [0 cgVelHeadY(i)])

    set(bodyHeadingLine, 'YData', [0 bodyHeadingX(i)])
    set(bodyHeadingLine, 'XData', [0 bodyHeadingY(i)])

    legend('CG Vel. Dir.', 'Body Heading Dir.')

    timeStr = sprintf('t = %.2f s', out.tout(i));
    set(an, 'String', timeStr)

    drawnow limitrate

end
%% IF USING LAPSIM DATA
figure
hold on
grid on
plot(Result.t,Result.v,'.-');
plot(out.bFrame.Speed)
xlabel('Time [s]')
ylabel('Speed [m/s]')
legend('David LapSim', 'Lateral Model')
