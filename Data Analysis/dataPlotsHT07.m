%% Inverters

figure
subplot(2,1,1)
hold on
plot(data.MOTOR_CONTROLLER.mc_fl.speed(:,1),data.MOTOR_CONTROLLER.mc_fl.speed(:,2).*0.2.*0.1047198./11.86,'.')
plot(data.MOTOR_CONTROLLER.mc_fr.speed(:,1),data.MOTOR_CONTROLLER.mc_fr.speed(:,2).*0.2.*0.1047198./11.86,'.')
plot(data.MOTOR_CONTROLLER.mc_rl.speed(:,1),data.MOTOR_CONTROLLER.mc_rl.speed(:,2).*0.2.*0.1047198./11.86,'.')
plot(data.MOTOR_CONTROLLER.mc_rr.speed(:,1),data.MOTOR_CONTROLLER.mc_rr.speed(:,2).*0.2.*0.1047198./11.86,'.')
legend("Front Left","Front Right","Rear Left","Rear Right")
xlabel('Time (s)')
ylabel('Speed (m/s)')

subplot(2,1,2)
hold on
plot(data.MOTOR_CONTROLLER.mc_fl.feedback_torque(:,1),data.MOTOR_CONTROLLER.mc_fl.feedback_torque(:,2),'.');
plot(data.MOTOR_CONTROLLER.mc_fr.feedback_torque(:,1),data.MOTOR_CONTROLLER.mc_fr.feedback_torque(:,2),'.');
% plot(data.MOTOR_CONTROLLER.mc_rl.feedback_torque(:,1),data.MOTOR_CONTROLLER.mc_rl.feedback_torque(:,2),'.');
% plot(data.MOTOR_CONTROLLER.mc_rr.feedback_torque(:,1),data.MOTOR_CONTROLLER.mc_rr.feedback_torque(:,2),'.');
legend("Front Left","Front Right","Rear Left","Rear Right")
xlabel('Time (s)')
ylabel('Torque (Nm)')
%% power

% plot(data.MOTOR_CONTROLLER.mc_fl.actual_power(:,1),data.MOTOR_CONTROLLER.mc_fl.actual_power(:,2))
% plot(data.MOTOR_CONTROLLER.mc_fr.actual_power(:,1),data.MOTOR_CONTROLLER.mc_fr.actual_power(:,2))
% plot(data.MOTOR_CONTROLLER.mc_rl.actual_power(:,1),data.MOTOR_CONTROLLER.mc_rl.actual_power(:,2))
% plot(data.MOTOR_CONTROLLER.mc_rr.actual_power(:,1),data.MOTOR_CONTROLLER.mc_rr.actual_power(:,2))

tMin = min(data.MOTOR_CONTROLLER.mc_fl.actual_power(:,1));
tMax = max(data.MOTOR_CONTROLLER.mc_fl.actual_power(:,1));
interpTime = tMin:0.005:tMax;

% Data uniqueness - averages same time data points
[uniqueTimePowerFL,~,idx] = unique(data.MOTOR_CONTROLLER.mc_fl.actual_power(:,1));
uniquePowerFL = accumarray(idx,data.MOTOR_CONTROLLER.mc_fl.actual_power(:,2),[],@mean);
[uniqueTimePowerFR,~,idx] = unique(data.MOTOR_CONTROLLER.mc_fr.actual_power(:,1));
uniquePowerFR = accumarray(idx,data.MOTOR_CONTROLLER.mc_fr.actual_power(:,2),[],@mean);
[uniqueTimePowerRL,~,idx] = unique(data.MOTOR_CONTROLLER.mc_rl.actual_power(:,1));
uniquePowerRL = accumarray(idx,data.MOTOR_CONTROLLER.mc_rl.actual_power(:,2),[],@mean);
[uniqueTimePowerRR,~,idx] = unique(data.MOTOR_CONTROLLER.mc_rr.actual_power(:,1));
uniquePowerRR = accumarray(idx,data.MOTOR_CONTROLLER.mc_rr.actual_power(:,2),[],@mean);

[uniqueTimeTorqueFL,~,idx] = unique(data.MOTOR_CONTROLLER.mc_fl.feedback_torque(:,1));
uniqueTorqueFL = accumarray(idx,data.MOTOR_CONTROLLER.mc_fl.feedback_torque(:,2),[],@mean);
[uniqueTimeTorqueFR,~,idx] = unique(data.MOTOR_CONTROLLER.mc_fr.feedback_torque(:,1));
uniqueTorqueFR = accumarray(idx,data.MOTOR_CONTROLLER.mc_fr.feedback_torque(:,2),[],@mean);
[uniqueTimeTorqueRL,~,idx] = unique(data.MOTOR_CONTROLLER.mc_rl.feedback_torque(:,1));
uniqueTorqueRL = accumarray(idx,data.MOTOR_CONTROLLER.mc_rl.feedback_torque(:,2),[],@mean);
[uniqueTimeTorqueRR,~,idx] = unique(data.MOTOR_CONTROLLER.mc_rr.feedback_torque(:,1));
uniqueTorqueRR = accumarray(idx,data.MOTOR_CONTROLLER.mc_rr.feedback_torque(:,2),[],@mean);

powerFL = interp1(uniqueTimePowerFL,uniquePowerFL,interpTime);
powerFR = interp1(uniqueTimePowerFR,uniquePowerFR,interpTime);
powerRL = interp1(uniqueTimePowerRL,uniquePowerRL,interpTime);
powerRR = interp1(uniqueTimePowerRR,uniquePowerRR,interpTime);

torqueFL = interp1(uniqueTimeTorqueFL,uniqueTorqueFL, interpTime);
torqueFR = interp1(uniqueTimeTorqueFR,uniqueTorqueFR, interpTime);
torqueRL = interp1(uniqueTimeTorqueRL,uniqueTorqueRL, interpTime);
torqueRR = interp1(uniqueTimeTorqueRR,uniqueTorqueRR, interpTime);

powerFL(torqueFL < 0) = -powerFL(torqueFL < 0);
powerFR(torqueFR < 0) = -powerFR(torqueFR < 0);
powerRL(torqueRL < 0) = -powerRL(torqueRL < 0);
powerRR(torqueRR < 0) = -powerRR(torqueRR < 0);

sumPower = powerFL + powerFR + powerRL + powerRR;

figure
hold on

plot(interpTime,powerFL)
plot(interpTime,powerFR)
plot(interpTime,powerRL)
plot(interpTime,powerRR)

plot(interpTime,sumPower)

legend("Front Left","Front Right","Rear Left","Rear Right","Total Power")
xlabel('Time (s)')
ylabel('Mechanical Motor Power Output (W)')

sumPower(isnan(sumPower)) = 0;
energy = cumtrapz(interpTime,sumPower);

figure
plot(interpTime,energy*2.77778e-7)
title('MECHANICAL POWER - NOT ACCUMULATOR ENERGY DRAW')
%% Motor Performance Curve
% Interpolate inverter data to common time axis
% tMin = min(data.MOTOR_CONTROLLER.mc_fl.speed(:,1));
% tMax = max(data.MOTOR_CONTROLLER.mc_fl.speed(:,1));
tMin = 244;
tMax = 744;
interpTime = tMin:0.05:tMax;

speedFLdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_fl.speed,interpTime);
torqueFLdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_fl.feedback_torque,interpTime);
speedFRdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_fr.speed,interpTime);
torqueFRdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_fr.feedback_torque,interpTime);
speedRLdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_rl.speed,interpTime);
torqueRLdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_rl.feedback_torque,interpTime);
speedRRdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_rr.speed,interpTime);
torqueRRdata = uniqueInterpolatedData(data.MOTOR_CONTROLLER.mc_rr.feedback_torque,interpTime);

figure
hold on

plot(speedRLdata.*0.2.*0.1047198./11.86,torqueRLdata,'.')
plot(speedRRdata.*0.2.*0.1047198./11.86,torqueRRdata,'.')
plot(speedFLdata.*0.2.*0.1047198./11.86,torqueFLdata,'.')
plot(speedFRdata.*0.2.*0.1047198./11.86,torqueFRdata,'.')
ylim([-25,25])
xlim([5,35])
xlabel('Wheel Ground Speed (m/s)')
ylabel('Torque (Nm)')
title('Vehicle Data, Derating Mode 2000 RPM Start Derate 6-10-23')
legend({'Rear Left','Rear Right','Front Left','Front Right'})

%% Energy Meter
figure
hold on
plot(data.ENERGY_METER.voltage(:,1),data.ENERGY_METER.voltage(:,2))
plot(data.ENERGY_METER.current(:,1),data.ENERGY_METER.current(:,2))
% plot(data.ENERGY_METER.overpower(:,1),data.ENERGY_METER.overpower(:,2))

%% Steering
figure
plot(data.MCU.analog.steering_2(:,1), -0.111*data.MCU.analog.steering_2(:,2) + 260)
legend('Steering Angle')

%% IMU
figure
hold on
% plot(data.IMU.long_accel(:,1),movmean(data.IMU.long_accel(:,2),5))
% plot(data.IMU.lat_accel(:,1),movmean(data.IMU.lat_accel(:,2),5))
% plot(data.IMU.vert_accel(:,1),movmean(data.IMU.vert_accel(:,2),5))

% figure
% hold on
plot(data.IMU.pitch(:,1),movmean(data.IMU.pitch(:,2),5))
% plot(data.IMU.roll(:,1),movmean(data.IMU.roll(:,2),5))
plot(data.IMU.yaw(:,1),movmean(data.IMU.yaw(:,2),5))

% figure
% yawAngle = cumtrapz(data.IMU.yaw(:,1),data.IMU.yaw(:,2));
% plot(data.IMU.yaw(:,1),yawAngle)

% figure
% rollAngle = cumtrapz(data.IMU.roll(:,1),data.IMU.roll(:,2));

% plot(data.IMU.roll(:,1),rollAngle)
legend('lat accel', 'ro angle')
pitch = cumtrapz(data.IMU.pitch(:,1),data.IMU.pitch(:,2));
figure
plot(data.IMU.pitch(:,1),pitch)
%% IMU Velocity Drift Analysis
tStart = 0;
tEnd = 350;
dt = 0.02; % 50 Hz
timeVelStudy = tStart:dt:tEnd;
% Extract Wheel Speed Data
studyWheelFLTime = data.MOTOR_CONTROLLER.mc_fl.speed(:,1);
studyWheelFLSpeed = data.MOTOR_CONTROLLER.mc_fl.speed(:,2);
studyWheelFRTime = data.MOTOR_CONTROLLER.mc_fr.speed(:,1);
studyWheelFRSpeed = data.MOTOR_CONTROLLER.mc_fr.speed(:,2);
studyWheelRLTime = data.MOTOR_CONTROLLER.mc_rl.speed(:,1);
studyWheelRLSpeed = data.MOTOR_CONTROLLER.mc_rl.speed(:,2);
studyWheelRRTime = data.MOTOR_CONTROLLER.mc_rr.speed(:,1);
studyWheelRRSpeed = data.MOTOR_CONTROLLER.mc_rr.speed(:,2);

% FL Ground Speed
maskWheelFL = studyWheelFLTime > tEnd + 0.5 | studyWheelFLTime < tStart - 0.5;
studyWheelFLTime(maskWheelFL) = [];
studyWheelFLSpeed(maskWheelFL) = [];
studyWheelFLGroundSpeed = studyWheelFLSpeed.*0.2.*0.1047198./11.86;
[uniqueStudyWheelFLTime,~,idx] = unique(studyWheelFLTime);
uniqueStudyWheelFLGroundSpeed = accumarray(idx,studyWheelFLGroundSpeed,[],@mean);
% FR Ground Speed
maskWheelFR = studyWheelFRTime > tEnd + 0.5 | studyWheelFRTime < tStart- 0.5;
studyWheelFRTime(maskWheelFR) = [];
studyWheelFRSpeed(maskWheelFR) = [];
studyWheelFRGroundSpeed = studyWheelFRSpeed.*0.2.*0.1047198./11.86;
[uniqueStudyWheelFRTime,~,idx] = unique(studyWheelFRTime);
uniqueStudyWheelFRGroundSpeed = accumarray(idx,studyWheelFRGroundSpeed,[],@mean);
% RL Ground Speed
maskWheelRL = studyWheelRLTime > tEnd + 0.5 | studyWheelRLTime < tStart- 0.5;
studyWheelRLTime(maskWheelRL) = [];
studyWheelRLSpeed(maskWheelRL) = [];
studyWheelRLGroundSpeed = studyWheelRLSpeed.*0.2.*0.1047198./11.86;
[uniqueStudyWheelRLTime,~,idx] = unique(studyWheelRLTime);
uniqueStudyWheelRLGroundSpeed = accumarray(idx,studyWheelRLGroundSpeed,[],@mean);
% RR Ground Speed
maskWheelRR = studyWheelRRTime > tEnd + 0.5 | studyWheelRRTime < tStart- 0.5;
studyWheelRRTime(maskWheelRR) = [];
studyWheelRRSpeed(maskWheelRR) = [];
studyWheelRRGroundSpeed = studyWheelRRSpeed.*0.2.*0.1047198./11.86;
[uniqueStudyWheelRRTime,~,idx] = unique(studyWheelRRTime);
uniqueStudyWheelRRGroundSpeed = accumarray(idx,studyWheelRRGroundSpeed,[],@mean);

studyAccelTime = data.IMU.long_accel(:,1);
studyAccelX = data.IMU.long_accel(:,2) + 1; % add 1 to account for static offset
maskAccel = studyAccelTime > tEnd + 0.5 | studyAccelTime < tStart- 0.5;
studyYawTime = data.IMU.yaw(:,1);
studyYaw = data.IMU.yaw(:,2);
maskYaw = studyYawTime > tEnd + 0.5 | studyYawTime < tStart - 0.5;
studyYawTime(maskYaw) = [];
studyYaw(maskYaw) = [];
studyAccelTime(maskAccel) = [];
studyAccelX(maskAccel) = [];

[uniqueStudyYawTime,~,idx] = unique(studyYawTime);
uniqueStudyYaw = accumarray(idx,studyYaw,[],@mean);

[uniqueStudyAccelTime,~,idx] = unique(studyAccelTime);
uniqueStudyAccelX = accumarray(idx,studyAccelX,[],@mean);

studyIMULongVel = cumtrapz(studyAccelTime,studyAccelX);
vehSpeed = [];

halfTrack = 0.6; % m
halfWheelbase = 1.535/2; % m
instFLSpeed = [];
instFRSpeed = [];
instRLSpeed = [];
instRRSpeed = [];
for ic = 1:length(timeVelStudy)
    instTime = timeVelStudy(ic);

    instAx = interp1(uniqueStudyAccelTime,uniqueStudyAccelX,instTime);
    instYawRad = deg2rad(interp1(uniqueStudyYawTime,uniqueStudyYaw,instTime));
    if isnan(instAx)
        instAx = 0;
    end
    % Positive yaw rate = left hand turn
    instFLSpeed(ic) = interp1(uniqueStudyWheelFLTime,uniqueStudyWheelFLGroundSpeed,instTime) + instYawRad.*halfTrack;
    instFRSpeed(ic) = interp1(uniqueStudyWheelFRTime,uniqueStudyWheelFRGroundSpeed,instTime) - instYawRad.*halfTrack;
    instRLSpeed(ic) = interp1(uniqueStudyWheelRLTime,uniqueStudyWheelRLGroundSpeed,instTime) + instYawRad.*halfTrack;
    instRRSpeed(ic) = interp1(uniqueStudyWheelRRTime,uniqueStudyWheelRRGroundSpeed,instTime) - instYawRad.*halfTrack;
    avgSpeed = (instFLSpeed(ic) + instFRSpeed(ic) + instRLSpeed(ic) + instRRSpeed(ic))/4;

    speedDiffFL = (instFLSpeed(ic) - avgSpeed)/avgSpeed;
    speedDiffFR = (instFRSpeed(ic) - avgSpeed)/avgSpeed;
    speedDiffRL = (instRLSpeed(ic) - avgSpeed)/avgSpeed;
    speedDiffRR = (instRRSpeed(ic) - avgSpeed)/avgSpeed;
    speedDiffVec = [speedDiffFL,speedDiffFR,speedDiffRL,speedDiffRR];
    speedCheck = sum(abs(speedDiffVec) < 0.05);

    if speedCheck == 4
        vehSpeed(ic) = avgSpeed;
    elseif ic == 1
        vehSpeed(ic) = dt*instAx;
    else
        vehSpeed(ic) = vehSpeed(ic-1) + dt*instAx;
    end
end

figure
hold on
plot(timeVelStudy,vehSpeed)
plot(uniqueStudyWheelFLTime,uniqueStudyWheelFLGroundSpeed,'.')
plot(uniqueStudyWheelFRTime,uniqueStudyWheelFRGroundSpeed,'.')
plot(uniqueStudyWheelRLTime,uniqueStudyWheelRLGroundSpeed,'.')
plot(uniqueStudyWheelRRTime,uniqueStudyWheelRRGroundSpeed,'.')

legend({'Integrated and Wheel-Speed Corrected IMU Velocity','Front Left Wheel Ground Speed','Front Right Wheel Ground Speed','Rear Left Wheel Ground Speed','Rear Right Wheel Ground Speed'})
xlabel('Time (s)')
ylabel('Velocity (m/s)')

figure
hold on
plot(timeVelStudy,vehSpeed)
plot(timeVelStudy,instFLSpeed,'.')
plot(timeVelStudy,instFRSpeed,'.')
plot(timeVelStudy,instRLSpeed,'.')
plot(timeVelStudy,instRRSpeed,'.')

title('Yaw Corrected Wheel Ground Speed')
legend({'Integrated and Wheel-Speed Corrected IMU Velocity','Front Left Wheel Ground Speed','Front Right Wheel Ground Speed','Rear Left Wheel Ground Speed','Rear Right Wheel Ground Speed'})
xlabel('Time (s)')
ylabel('Velocity (m/s)')


%% Temperatures

figure
subplot(3,1,1)
hold on
plot(data.MOTOR_CONTROLLER.mc_fl.igbt_temp(:,1),data.MOTOR_CONTROLLER.mc_fl.igbt_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_fr.igbt_temp(:,1),data.MOTOR_CONTROLLER.mc_fr.igbt_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_rl.igbt_temp(:,1),data.MOTOR_CONTROLLER.mc_rl.igbt_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_rr.igbt_temp(:,1),data.MOTOR_CONTROLLER.mc_rr.igbt_temp(:,2))
title(['IGBT Temperature (125' char(176) 'C) Max'])
xlabel('Time (s)')
ylabel(['Temperature (' char(176) 'C)'])
legend({'Front Left','Front Right','Rear Left','Rear Right'})

subplot(3,1,2)
hold on
plot(data.MOTOR_CONTROLLER.mc_fl.inverter_temp(:,1),data.MOTOR_CONTROLLER.mc_fl.inverter_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_fr.inverter_temp(:,1),data.MOTOR_CONTROLLER.mc_fr.inverter_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_rl.inverter_temp(:,1),data.MOTOR_CONTROLLER.mc_rl.inverter_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_rr.inverter_temp(:,1),data.MOTOR_CONTROLLER.mc_rr.inverter_temp(:,2))
title(['Cold Plate Temperature (60' char(176) 'C) Max'])
xlabel('Time (s)')
ylabel(['Temperature (' char(176) 'C)'])
legend({'Front Left','Front Right','Rear Left','Rear Right'})

subplot(3,1,3)
hold on
plot(data.MOTOR_CONTROLLER.mc_fl.motor_temp(:,1),data.MOTOR_CONTROLLER.mc_fl.motor_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_fr.motor_temp(:,1),data.MOTOR_CONTROLLER.mc_fr.motor_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_rl.motor_temp(:,1),data.MOTOR_CONTROLLER.mc_rl.motor_temp(:,2))
plot(data.MOTOR_CONTROLLER.mc_rr.motor_temp(:,1),data.MOTOR_CONTROLLER.mc_rr.motor_temp(:,2))
title(['Motor Temperature (140' char(176) 'C) Max'])
xlabel('Time (s)')
ylabel(['Temperature (' char(176) 'C)'])
legend({'Front Left','Front Right','Rear Left','Rear Right'})
ylim([20,150])

figure
hold on
plot(data.BMS.temperatures.high_temperature(:,1),data.BMS.temperatures.high_temperature(:,2))
plot(data.BMS.temperatures.average_temperature(:,1),data.BMS.temperatures.average_temperature(:,2))
plot(data.BMS.temperatures.low_temperature(:,1),data.BMS.temperatures.low_temperature(:,2))

title(['Accumulator Cell Temperatures (60' char(176) 'C) Max'])
xlabel('Time (s)')
ylabel(['Temperature (' char(176) 'C)'])
legend({'High Temperature','Average Temperature','Low Temperature'})
% plot(data.BMS.detailed_temps.ic_00.temp_1(:,1),data.BMS.detailed_temps.ic_00.temp_1(:,2))

%% Accumulator Voltages
figure
plot(data.BMS.detailed_voltage.ic_00.cell_01(:,1),data.BMS.detailed_voltage.ic_00.cell_01(:,2))

%% Load cell aero analysis

startTime = 33;
endTime = 38;

interpTime = data.MCU.load_cells.FL(:,1);
[uniqueTimeSpeedFR,~,idx] = unique(data.MOTOR_CONTROLLER.mc_fr.speed(:,1));
uniqueSpeedFR = accumarray(idx,data.MOTOR_CONTROLLER.mc_fr.speed(:,2),[],@mean);
[uniqueTimeSpeedFL,~,idx] = unique(data.MOTOR_CONTROLLER.mc_fl.speed(:,1));
uniqueSpeedFL = accumarray(idx,data.MOTOR_CONTROLLER.mc_fl.speed(:,2),[],@mean);
uniqueLongAccel = uniqueData(data.IMU.long_accel);
uniqueLatAccel = uniqueData(data.IMU.lat_accel);

interpLongAccel = interp1(uniqueLongAccel(:,1),uniqueLongAccel(:,2),interpTime);
interpLatAccel = interp1(uniqueLatAccel(:,1),uniqueLatAccel(:,2),interpTime);
interpSpeedFL = interp1(uniqueTimeSpeedFL,uniqueSpeedFL,interpTime).*0.2.*0.1047198./11.86; % m/s
interpSpeedFR = interp1(uniqueTimeSpeedFR,uniqueSpeedFR,interpTime).*0.2.*0.1047198./11.86; % m/s
vehicleSpeed = (interpSpeedFR + interpSpeedFL)./2;

frontLoadCellForce = data.MCU.load_cells.FL(:,2) + data.MCU.load_cells.FR(:,2);
rearLoadCellForce = data.MCU.load_cells.RL(:,2) + data.MCU.load_cells.RR(:,2);

frontLoadCellForce(interpLatAccel > 1 | interpLongAccel > 3) = 0;
rearLoadCellForce(interpLatAccel > 1 | interpLongAccel > 3) = 0;

figure
plot(vehicleSpeed,frontLoadCellForce+rearLoadCellForce,'.')

figure
plot(data.MCU.load_cells.FL(:,1), data.MCU.load_cells.FL(:,2)+ data.MCU.load_cells.FR(:,2)+ data.MCU.load_cells.RL(:,2)+ data.MCU.load_cells.RR(:,2))
%%
figure
hold on
plot(interpTime, frontLoadCellForce)
plot(interpTime, rearLoadCellForce)
hold off
histogram(frontLoadCellForce, 40)
histogram(rearLoadCellForce, 40)
%% GPS
gpsLat = data.MCU.GPS.latitude(:,2);
gpsLong = data.MCU.GPS.longitude(:,2);
gpsAcc = data.MCU.GPS.accuracy(:,2);
gpsTime = gpsLat(:,1);

mask = gpsAcc > 2;
gpsAcc(mask) = [];
gpsTime(mask) = [];
gpsLat(mask) = [];
gpsLong(mask) = [];

figure
plot(gpsLat,-gpsLong);

% interpTime = min(data.MCU.GPS.latitude(:,1)

%% Load Cell
figure
hold on
plot(data.MCU.load_cells.FL(:,1),data.MCU.load_cells.FL(:,2))
plot(data.MCU.load_cells.FR(:,1),data.MCU.load_cells.FR(:,2))
plot(data.MCU.load_cells.RL(:,1),data.MCU.load_cells.RL(:,2))
plot(data.MCU.load_cells.RR(:,1),data.MCU.load_cells.RR(:,2))
legend("Front Left","Front Right","Rear Left","Rear Right")

%% Damper Pots
figure
hold on
plot(data.MCU.pots.pot1(:,1),data.MCU.pots.pot1(:,2)) % FL
% plot(data.MCU.pots.pot2(:,1),data.MCU.pots.pot2(:,2)-data.MCU.pots.pot2(end,2))
plot(data.MCU.pots.pot3(:,1),data.MCU.pots.pot3(:,2)) % FR
plot(data.MCU.pots.pot4(:,1),data.MCU.pots.pot4(:,2)) % RL
% plot(data.MCU.pots.pot5(:,1),data.MCU.pots.pot5(:,2)-data.MCU.pots.pot5(end,2))
plot(data.MCU.pots.pot6(:,1),data.MCU.pots.pot6(:,2)) % RR

%% Cell Data
figure
hold on
plot(data.BMS.voltages.average_voltage(:,1),data.BMS.voltages.average_voltage(:,2))
plot(data.BMS.voltages.high_voltage(:,1),data.BMS.voltages.high_voltage(:,2))
plot(data.BMS.voltages.low_voltage(:,1),data.BMS.voltages.low_voltage(:,2))

figure

for ic = 0:11
    subplot(3,4,ic+1)
    title(['IC ',num2str(ic)])
    hold on
    if length(num2str(ic)) == 1
        icStr = ['ic_0',num2str(ic)];
    else
        icStr = ['ic_',num2str(ic)];
    end
    if mod(ic,2) == 0
        for cell = 1:12
            if length(num2str(cell)) == 1
                cellStr = ['cell_0',num2str(cell)];
            else
                cellStr = ['cell_',num2str(cell)];
            end
            plot(data.BMS.detailed_voltage.(icStr).(cellStr)(:,1),data.BMS.detailed_voltage.(icStr).(cellStr)(:,2))
        end
    else
        for cell = 1:9
            if length(num2str(cell)) == 1
                cellStr = ['cell_0',num2str(cell)];
            else
                cellStr = ['cell_',num2str(cell)];
            end
            plot(data.BMS.detailed_voltage.(icStr).(cellStr)(:,1),data.BMS.detailed_voltage.(icStr).(cellStr)(:,2))
        end
    end
end

%% Load cell suspension analysis

startTime = 0;
endTime = 500;

interpTime = data.MCU.load_cells.FL(:,1);
[uniqueTimeLatAccel,~,idx] = unique(data.IMU.lat_accel(:,1));
uniqueLatAccel = accumarray(idx,data.IMU.lat_accel(:,2),[],@mean);

interpLatAccel = interp1(uniqueTimeLatAccel,uniqueLatAccel,interpTime);

frontLoadCellTransfer = data.MCU.load_cells.FL(:,2) - data.MCU.load_cells.FR(:,2);
rearLoadCellTransfer = data.MCU.load_cells.RL(:,2) - data.MCU.load_cells.RR(:,2);
mask = interpTime < startTime | interpTime > endTime;
interpTime(mask) = [];
frontLoadCellTransfer(mask) = [];
rearLoadCellTransfer(mask) = [];
interpLatAccel(mask) = [];

figure
hold on
plot(movmean(interpLatAccel,20)./9.81,movmean(frontLoadCellTransfer*4.44822,20),'.')
plot(movmean(interpLatAccel,20)./9.81,movmean(rearLoadCellTransfer*4.44822,20),'.')
xlabel('Lateral Acceleration (g)')
ylabel('Lateral Load Transfer (N)')

function [timeAdjustedData] = uniqueInterpolatedData(timeSeriesData,newTimeVector)

[lines, ~, subs] = unique(timeSeriesData(:,1));
ymeanperline = accumarray(subs, timeSeriesData(:,2), [], @mean);
timeAdjustedData = interp1(lines,ymeanperline,newTimeVector);

end