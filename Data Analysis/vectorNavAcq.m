%% CONNECT TO SENSOR
clear;
close all;
clc;
NET.addAssembly('C:\Users\lukec\OneDrive - Georgia Institute of Technology\HyTech\MATLAB\vnproglib\net\bin\win32\VectorNav.dll');

import VectorNav.Sensor.*
import VectorNav.Protocol.Uart.* 

ez = EzAsyncData.Connect('COM3', 115200);
%% 
ez.CurrentData.YawPitchRoll
%% 
ez.CurrentData.AngularRateUncompensated
%% 
native = [native single(ez.CurrentData.YawPitchRoll.ToArray())];

%% 
br = ez.Sensor.ReadBinaryOutput1();
%% 
br.RateDivisor = 100;
br.AsyncMode = VectorNav.Protocol.Uart.AsyncMode.Port1;
%% 
% test = VectorNav.Protocol.Uart.CommonGroup;
br.CommonField = bitor(bitor(VectorNav.Protocol.Uart.CommonGroup.TimeStartup, VectorNav.Protocol.Uart.CommonGroup.YawPitchRoll), VectorNav.Protocol.Uart.CommonGroup.Imu)
%% 
ez.Sensor.WriteBinaryOutput1(br);
%% 

yawPitchRollVec = [];
accelVec = [];
fprintf('Calling ez.GetNextData()\n');
for n = 1:500
    cd = ez.GetNextData(250);
    % fprintf('YPR: %3.2f %3.2f %3.2f\n', cd.YawPitchRoll.X, cd.YawPitchRoll.Y, cd.YawPitchRoll.Z);
    % yawPitchRollVec = [yawPitchRollVec; [cd.YawPitchRoll.X cd.YawPitchRoll.Y cd.YawPitchRoll.Z]];
    % fprintf('YPR: %3.2f %3.2f %3.2f\n', cd.AccelerationUncompensated.X, cd.AccelerationUncompensated.Y, cd.AccelerationUncompensated.Z);
    % accelVec = [accelVec; [cd.AccelerationUncompensated.X cd.AccelerationUncompensated.Y cd.AccelerationUncompensated.Z]];


end
%% 
noiseMask = abs(accelVec) < 0.05;
accelVec(noiseMask) = 0;
%% 
figure
hold on
plot(accelVec(:,1))
plot(accelVec(:,2))
plot(accelVec(:,3))
legend('X', 'Y', 'Z')

%% 
velVec = [];
m_s2mph =  2.237;
% fprintf('Calling ez.GetNextData()\n');
figure
for n = 1:inf
    cd = ez.CurrentData;

    % fprintf('%3.2f\n', cd.VelocityEstimatedBody.X);

    fprintf('Vel: x: %3.2f [MPH] y: %3.2f [MPH] z: %3.2f\n [MPH]', ...
        cd.VelocityEstimatedBody.X .* m_s2mph, cd.VelocityEstimatedBody.Y .* m_s2mph, cd.VelocityEstimatedBody.Z .* m_s2mph);
    velVec = [velVec; [cd.VelocityEstimatedBody.X, cd.VelocityEstimatedBody.Y, cd.VelocityEstimatedBody.Z]];
    
    subplot(2,1,1)
    grid on
    box on
    plot(cd.VelocityEstimatedBody.X .* m_s2mph, 'o')
    legend('Velocity X (MPH)')
    ylim([-10 50])
    drawnow limitrate

    subplot(2,1,2)
    grid on
    box on
    plot(cd.VelocityEstimatedBody.Y .* m_s2mph, 'o')
    legend('Velocity Y (MPH)')
    ylim([-2 2])
    drawnow limitrate

end
%% 

velMask = abs(velVec) < 0.5;
velVec(velMask) = 0;

%% 
figure
hold on
plot(velVec(:,1))
plot(velVec(:,2))
plot(velVec(:,3))
legend('X', 'Y', 'Z')

%% 
slip = atand(velVec(:,2)./velVec(:,1));
slip(isnan(slip)) = 0;

figure

hold on
plot(slip)
%% 
velVec = [];
% fprintf('Calling ez.GetNextData()\n');
for n = 1:inf
    cd = ez.CurrentData;

    % fprintf('Vel: %3.2f %3.2f %3.2f\n', cd.VelocityEstimatedBody.X, cd.VelocityEstimatedBody.Y, cd.VelocityEstimatedBody.Z);
    velVec = [velVec; [cd.VelocityEstimatedBody.X, cd.VelocityEstimatedBody.Y, cd.VelocityEstimatedBody.Z]];
    velMask = abs(velVec) < 0.2;
    velVec(velMask) = 0;
    slip = atand(velVec(:,2)./velVec(:,1));
    slipMask = slip > 20;
    slip(slipMask) = 0;
    slip(isnan(slip)) = 0;

    subplot(3,1,1)
    plot(velVec(:,1))
    title('Velocity X (m/s)')
    drawnow limitrate

    subplot(3,1,2)
    plot(velVec(:,2))
    title('Velocity Y (m/s)')
    drawnow limitrate

    subplot(3,1,3)
    plot(slip)
    title('Body Slip (deg)')
    drawnow limitrate

    % fprintf('Slip: %3.2f\n', slip(end));
end
%% 
figure

hold on
plot(slip)

%% NO GNSS FIX
close all
angVelVec = [];
timeVec = [];
hz = 20;
nanoS2S = 1/(1e9);
for n = 1:inf
    pause(1/hz)
    cd = ez.CurrentData;
    timeVec = [timeVec; cd.TimeStartup * nanoS2S];
    angVelVec = [angVelVec; [cd.AngularRate.X cd.AngularRate.Y cd.AngularRate.Z]];

    subplot(3,1,1)
    plot((cd.AngularRate.X), 'o')
    ylim([-5 5])
    title('Angular Rate X (rad/s)')

    subplot(3,1,2)
    plot((cd.AngularRate.Y), 'o')
    ylim([-5 5])
    title('Angular Rate Y (rad/s)')

    subplot(3,1,3)
    plot((cd.AngularRate.Z), 'o')
    ylim([-5 5])
    title('Angular Rate Z (rad/s)')

    drawnow

end
%% DATA LOGGING
timeToLog = 6000; % seconds
rate = 20; % Hz
allocateSize = timeToLog / (1/rate);

timeStartUp = ones(allocateSize, 1);
bodyLinVel = zeros(allocateSize, 3);
bodyAngVel = zeros(allocateSize, 3);
bodyCompAccel = zeros(allocateSize, 3);

for n = 1:allocateSize

    pause(1/rate);

    cd = ez.CurrentData;
    
    timeStartUp(n, 1) = cd.TimeStartup;

    bodyLinVel(n, :) = [cd.VelocityEstimatedBody.X cd.VelocityEstimatedBody.Y cd.VelocityEstimatedBody.Z];

    bodyAngVel(n, :) = [cd.AngularRate.X cd.AngularRate.Y cd.AngularRate.Z];

    bodyCompAccel(n, :) = [cd.Acceleration.X cd.Acceleration.Y cd.Acceleration.Z];

end

%% TIME CONVERSION AND DATA RANGE
nanoS2s = 1e-9;

timeElapsed = (timeStartUp - timeStartUp(1)) .* nanoS2s;

endInd = find(timeElapsed < 0, 1) - 1;

timeElapsed = timeElapsed(1:endInd, :);
bodyLinVel = bodyLinVel(1:endInd, :);
bodyAngVel = bodyAngVel(1:endInd, :);
bodyCompAccel = bodyCompAccel(1:endInd, :);

%% PLOT LOGGED DATA
tiledlayout(3, 3)

nexttile
hold on
grid on
plot(timeElapsed, bodyLinVel(:, 1), 'r')
ylabel('[m/s]')
legend('Body Velocity X')

nexttile
hold on
grid on
plot(timeElapsed, bodyLinVel(:, 2), 'g')
ylabel('[m/s]')
legend('Body Velocity Y')

nexttile
hold on
grid on
plot(timeElapsed, bodyLinVel(:, 3), 'b')
ylabel('[m/s]')
legend('Body Velocity Z')

nexttile
hold on
grid on
plot(timeElapsed, bodyAngVel(:, 1), 'r')
ylabel('[rad/s]')
legend('Body Ang. Vel. X')

nexttile
hold on
grid on
plot(timeElapsed, bodyAngVel(:, 2), 'g')
ylabel('[rad/s]')
legend('Body Ang. Vel. Y')

nexttile
hold on
grid on
plot(timeElapsed, bodyAngVel(:, 3), 'b')
ylabel('[rad/s]')
legend('Body Ang. Vel. Z')

nexttile
hold on
grid on
plot(timeElapsed, bodyCompAccel(:, 1), 'r')
ylabel('[m/s^2]')
legend('Body Comp. Accel. X')

nexttile
hold on
grid on
plot(timeElapsed, bodyCompAccel(:, 2), 'g')
ylabel('[m/s^2]')
legend('Body Comp. Accel. Y')

nexttile
hold on
grid on
plot(timeElapsed, bodyCompAccel(:, 3), 'b')
ylabel('[m/s^2]')
legend('Body Comp. Accel. Z')

%% 
ez.Disconnect();