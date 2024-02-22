NET.addAssembly('C:\Users\lukec\OneDrive - Georgia Institute of Technology\HyTech\MATLAB\vnproglib\net\bin\win32\VectorNav.dll');

import VectorNav.Sensor.*
import VectorNav.Protocol.Uart.*
%% 

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
% fprintf('Calling ez.GetNextData()\n');
figure
for n = 1:inf
    cd = ez.CurrentData;

    % fprintf('%3.2f\n', cd.VelocityEstimatedBody.X);

    fprintf('Vel: %3.2f %3.2f %3.2f\n', cd.VelocityEstimatedBody.X, cd.VelocityEstimatedBody.Y, cd.VelocityEstimatedBody.Z);
    velVec = [velVec; [cd.VelocityEstimatedBody.X, cd.VelocityEstimatedBody.Y, cd.VelocityEstimatedBody.Z]];
    
    subplot(2,1,1)
    plot(cd.VelocityEstimatedBody.X)
    legend('Velocity X (m/s)')
    drawnow

    subplot(2,1,2)
    plot(cd.VelocityEstimatedBody.Y)
    legend('Velocity Y (m/s)')
    drawnow

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
%% 
ez.Disconnect();