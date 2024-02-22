close all
clear;
clc;


%% Vehicle Param Estimate (STATIC)
HT07_vehicle_parameters;
MR_fr = 0.938;
MR_rr = 0.84;
track = 1.2; % m
cgz = 0.2;
roRate_fr = 15600; % Nm/rad
roRate_rr = 23000; % Nm/rad

mass = [70.397536, 60.736018;
        58.195901, 68.583166];

mass_fr = mass(1)+mass(3);
mass_rr = mass(2)+mass(4);
mass_tot = mass_fr+mass_rr; % kg, with driver + helmet

uMass = [7.3, 7.3;
         7.9, 7.9];
uMass_fr = uMass(1)+uMass(3);
uMass_rr = uMass(2)+uMass(4);

sMass = mass-uMass;
sMass_fr = sMass(1)+sMass(3);
sMass_rr = sMass(2)+sMass(4);
sMass_tot = sMass_fr+sMass_rr;

hf = 0.195; % m, front unsprung mass cg, estimated
hr = 0.195; % m, rear unsprung mass cg, estimated

hf_rc = 0.083699; % m, front roll center height
hr_rc = 0.064906; % m, rear roll center height

rr2cg = mass_fr/mass_tot*Parameters.L;
fr2cg = Parameters.L-rr2cg;

wb = Parameters.L; % m, wheelbase
b = Parameters.b; % m, distance from CG to rear axle

cg2RoAxis = cgz-abs(((hr_rc-hf_rc)/wb)*(wb-rr2cg)+hf_rc); % m, center of gravity to roll axis
cg2RrAxle = .78; % m, center of gravity to rear axle
cp2RrAxle = 0.8109; % m, center of pressure to rear axle
cpz = 0; % m, center of pressure height

g = 9.81; %m/2^2

rho = Parameters.rho; % Density of air
Cd = Parameters.Cd; % Drag coefficient
Cl = Parameters.Cl; % Lift coefficient
A = Parameters.A; % Reference frontal area

%% Data Pull and Time Shift
% Fitting everything to potRL time

% General Time
time = data.MCU.pots.pot4(:,1)'; %RL

% Pot Data Pull
potFL = data.MCU.pots.pot1(:,2); % FL
potFR = data.MCU.pots.pot3(:,2); % FR
potRL = data.MCU.pots.pot4(:,2)'; % RL
potRR = data.MCU.pots.pot6(:,2)'; % RR

originalSpacing = linspace(0,1,length(potFL));

% New Time Based on potRL
n = length(potRL);
newSpacing = linspace(0,1,n);

potFL = interp1(originalSpacing, potFL, newSpacing, 'spline');
potFR = interp1(originalSpacing, potFR, newSpacing, 'spline');

% Steering Data Pull
steering = -0.111*data.MCU.analog.steering_2(:,2) + 260; % deg
originalSpacing = linspace(0,1,length(steering));
steering = interp1(originalSpacing, steering, newSpacing, 'spline');

% IMU

IMU_time = data.IMU.long_accel(:,1);
IMU_timeMask = IMU_time < time(1) | IMU_time > time(end);

IMU_longAccel = data.IMU.long_accel(:,2).*0.5;
IMU_longAccel(IMU_timeMask) = [];
originalSpacing = linspace(0,1,length(IMU_longAccel));
IMU_longAccel = interp1(originalSpacing, IMU_longAccel, newSpacing, 'spline');

IMU_longAccel = filloutliers(IMU_longAccel, 'nearest', 'mean');
IMU_longAccel = smoothdata(IMU_longAccel, 'sgolay');

IMU_longG = IMU_longAccel./9.81;

IMU_roRate = data.IMU.roll(:,2);
IMU_roRate(IMU_timeMask) = [];
originalSpacing = linspace(0,1,length(IMU_roRate));
IMU_roRate = interp1(originalSpacing, IMU_roRate, newSpacing, 'spline');

IMU_yawRate = movmean(data.IMU.yaw(:,2), 5);
IMU_yawRate(IMU_timeMask) = [];
originalSpacing = linspace(0,1,length(IMU_yawRate));
IMU_yawRate = interp1(originalSpacing, IMU_yawRate, newSpacing, 'spline');

% Inverter
inverter_time = data.MOTOR_CONTROLLER.mc_fl.speed(:,1)';
inverter_timeMask = inverter_time < time(1) | inverter_time > time(end);

mc_FL = data.MOTOR_CONTROLLER.mc_fl.speed(:,2)'.*0.2.*0.1047198./11.86;
mc_FL(inverter_timeMask) = [];
originalSpacing = linspace(0,1,length(mc_FL));
mc_FL = interp1(originalSpacing, mc_FL, newSpacing, 'spline');

mc_FR = data.MOTOR_CONTROLLER.mc_fr.speed(:,2)'.*0.2.*0.1047198./11.86;
mc_FR(inverter_timeMask) = [];
originalSpacing = linspace(0,1,length(mc_FR));
mc_FR = interp1(originalSpacing, mc_FR, newSpacing, 'spline');

mc_RL = data.MOTOR_CONTROLLER.mc_rl.speed(:,2)'.*0.2.*0.1047198./11.86;
mc_RL(inverter_timeMask) = [];
originalSpacing = linspace(0,1,length(mc_RL));
mc_RL = interp1(originalSpacing, mc_RL, newSpacing, 'spline');

mc_RR = data.MOTOR_CONTROLLER.mc_rr.speed(:,2)'.*0.2.*0.1047198./11.86;
mc_RR(inverter_timeMask) = [];
originalSpacing = linspace(0,1,length(mc_RR));
mc_RR = interp1(originalSpacing, mc_RR, newSpacing, 'spline');

mc_avg_front = (mc_FL+mc_FR)./2;
mc_avg_rear = (mc_RL+mc_RR)./2;

%% POT PLOTS
close all;

figure
tiledlayout(3,1)
nexttile
hold on
plot(time, potFL);
plot(time, potFR);
plot(time, potRL);
plot(time, potRR);
legend('FL OG', 'FR OG', 'RL OG', 'RR OG')

FLrange = [0, 54];
FLmap = [3.11862, 243.688]; 
potFL = interp1(FLmap, FLrange, potFL);

FRrange = [0, 54.4];
FRmap = [3.11862, 242.883];
potFR = interp1(FRmap, FRrange, potFR);

RLrange = [0, 54.05];
RLmap = [3, 245.883];
potRL = interp1(RLmap, RLrange, potRL);

RRrange = [0, 54.5];
RRmap = [3.3, 245.883];
potRR = interp1(RRmap, RRrange, potRR);

nexttile
hold on
plot(time, potFL);
plot(time, potFR);
plot(time, potRL);
plot(time, potRR);
legend('FL OG (mm)', 'FR OG (mm)', 'RL OG (mm)', 'RR OG (mm)')

% potFL = potFL-mode(potFL);
% potFR = potFR-mode(potFR);
% potRL = potRL-mode(potRL);
% potRR = potRR-mode(potRR);

[N, edges] = histcounts(potFL(1:round(end/4)));
[~, ind] = max(N);
potFL = potFL - edges(ind);

[N, edges] = histcounts(potFR(1:round(end/4)));
[~, ind] = max(N);
potFR = potFR - edges(ind);

[N, edges] = histcounts(potRL(1:round(end/4)));
[~, ind] = max(N);
potRL = potRL - edges(ind);

[N, edges] = histcounts(potRR(1:round(end/4)));
[~, ind] = max(N);
potRR = potRR - edges(ind);

nexttile
hold on
plot(time, potFL);
plot(time, potFR);
plot(time, potRL);
plot(time, potRR);
legend('FL SHIFTED (mm)', 'FR SHIFTED (mm)', 'RL SHIFTED (mm)', 'RR SHIFTED (mm)')

potFL = filloutliers(potFL, 'nearest', 'mean');
potFR = filloutliers(potFR, 'nearest', 'mean');
potRL = filloutliers(potRL, 'nearest', 'mean');
potRR = filloutliers(potRR, 'nearest', 'mean');

% [~, outRL] = rmoutliers(potRL);
% [~, outRR] = rmoutliers(potRR);
% 
% loc = outRL | outRR;
% loc = outRR;
% 
% potRR(loc, :) = [];
% potRL(loc, :) = [];
% time(loc, :) = [];

potFL(potFL < 0.7 & potFL > -0.7) = 0;
potFR(potFR < 0.7 & potFR > -0.7) = 0;
potRL(potRL < 0.7 & potRL > -0.7) = 0;
potRR(potRR < 0.7 & potRR > -0.7) = 0;

% potFL = smoothdata(smoothdata((potFL), 'sgolay'), 'sgolay');
potFL = smoothdata((potFL), 'sgolay');
potFR = smoothdata((potFR), 'sgolay');
potRL = smoothdata((potRL), 'sgolay');
potRR = smoothdata((potRR), 'sgolay');

% 
% 
% potFL = potFR + (potRL-potRR);
% 
% 

% Fs = 50;
% Fn = Fs/2;
% [pn, tn] = resample(potFL, time, Fs);
% L = numel(tn);
% FTpn = fft(pn)/L;
% Fv = linspace(0,1,fix(L/2)+1)*Fn;
% Iv = 1:numel(Fv);
% 
% figure
% plot(Fv, abs(FTpn(Iv))*2)
% 
% Wp = 0.2/Fn;
% Ws = 0.5/Fn;
% Rp = 1;
% Rs = 60;
% [n,Wp] = ellipord(Wp,Ws,Rp,Rs);                                             % Elliptic Order Calculation
% [z,p,k] = ellip(n,Rp,Rs,Wp,'high');                                         % Elliptic Filter Design: Zero-Pole-Gain 
% [sos,g] = zp2sos(z,p,k);                                                    % Second-Order Section For Stability
% 
% figure
% freqz(sos, 2^16, Fs)   
% 
% potFL = filtfilt(sos,g, pn);  
% 
% figure
% plot(tn, pn_filt)



% potFR = potFR.*test;
% potRL = potRL.*test1;
% potRR = potRR.*test2;

% potRR = smoothdata(smoothdata(smoothdata(smoothdata(potRR))),'rloess');

% [~, ~, S2] = ischange(potRL);
% potRL = potRL - mode(S2);
% 
% [~, ~, S2] = ischange(potRR);
% potRR = potRR - mode(S2);

figure
tiledlayout(3,1);
tile1 = nexttile;
hold on
plot(time, potFL, '-.');
plot(time, potFR, '-.');
plot(time, potRL, '.-');
plot(time, potRR, '.-');
legend('FL Processed Displacement (mm)', 'FR Processed Displacement (mm)', ...
       'RL Processed Displacement (mm)', 'RR Processed Displacement (mm)', ...
       'AutoUpdate','off');


steering = filloutliers(steering, 'nearest', 'mean');
steering = smoothdata(steering, 'sgolay');

[TF, ~, ~] = ischange(steering, 'linear', 'Threshold', 300);
cpts = find(TF);

tile2 = nexttile;
hold on
plot(time, steering)
plot(time(cpts), steering(cpts), '^r', 'MarkerFaceColor','r')
legend('Steering Angle', 'Delta Points', 'AutoUpdate','off')

frontRoAng = -atand((((potFL)-(potFR))*MR_fr/(track*1000)));
rearRoAng = -atand((((potRL)-(potRR))*MR_rr/(track*1000)));
% roAng = cumsum(roAngInc);
frontLatG = (deg2rad(frontRoAng)*(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis))/(mass_tot*g*cg2RoAxis);
rearLatG = (deg2rad(rearRoAng)*(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis))/(mass_tot*g*cg2RoAxis);

% roAngRaw = atand(((diff(data.MCU.pots.pot4(:,2))-diff(data.MCU.pots.pot6(:,2)))*MR_rr/track));
% latGRaw = (deg2rad(roAngRaw)*(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis))/(mass_tot*g*cg2RoAxis);

tile3 = nexttile;
hold on
plot(time, frontRoAng)
plot(time, frontLatG)
plot(time, rearRoAng)
plot(time, rearLatG)
% plot(time(2:end), roAng)
% plot(time(2:end), latG)
% plot(time(2:end), smoothdata(smoothdata(smoothdata(latG), 'rloess'), 'sgolay'))
% plot(time(2:end), roAngRaw)
% plot(time(2:end), latGRaw)
plot(data.IMU.lat_accel(:,1),-movmean(data.IMU.lat_accel(:,2)./g,5))
legend('Front Roll Angle', 'Front LatG Calc', 'Rear Roll Angle', 'Rear LatG Calc', 'LatG IMU', 'AutoUpdate', 'off')

linkaxes([tile1, tile2, tile3], 'x')
zoom xon

% 
% figure
% hold on
% plot(time(2:end), roAngS)
% plot(time(2:end), latG_sm)
% legend('ro ang smooth', 'calculated latG smooth')
%% Roll Angle vs LatG (Roll Stiffness Distribution, lower slope = stiffer)

%WIP

figure

subplot(2,1,1)
plot(frontLatG, frontRoAng)
coeffs = polyfit(frontLatG, frontRoAng, 1);
xl = xlim;
yl = ylim;
xt = 0.05 * (xl(2)-xl(1)) + xl(1);
yt = 0.90 * (yl(2)-yl(1)) + yl(1);
txt = ['y = (' num2str(coeffs(1)) ')x + (' num2str(coeffs(2)) ')'];
text(xt,yt,txt)
title('Front Roll Angle and Front Lat G')

subplot(2,1,2)
plot(rearLatG, rearRoAng)
coeffs = polyfit(rearLatG, rearRoAng, 1);
xl = xlim;
yl = ylim;
xt = 0.05 * (xl(2)-xl(1)) + xl(1);
yt = 0.90 * (yl(2)-yl(1)) + yl(1);
txt = ['y = (' num2str(coeffs(1)) ')x + (' num2str(coeffs(2)) ')'];
text(xt,yt,txt)
title('Rear Roll Angle and Rear Lat G')

%% Draw Steer Delta Sections

for i = 1:3
    tile = nexttile(i);
    hold on
    xline(tile, data.MCU.analog.steering_2(cpts));
end

%% 

figure
hold on

plot(time, IMU_roRate)

IMU_roRate_modeShift = IMU_roRate-mode(IMU_roRate);

plot(time, IMU_roRate_modeShift)

[~, S1, S2] = ischange(IMU_roRate);
IMU_roRate_ischangeShift = IMU_roRate - mode(S1) - mode(S2);

plot(time, IMU_roRate_ischangeShift)

legend('Ro Rate OG', 'Ro Rate Mode Shifted', 'Ro Rate ischange Shifted')
%% 

IMU_roRate_ischangeShift = smoothdata(smoothdata(IMU_roRate_ischangeShift), 'sgolay');

figure
plot(time, IMU_roRate_ischangeShift)

%% 
figure

roAng_IMU = cumtrapz(time, IMU_roRate_ischangeShift);

plot(time, roAng_IMU)

% figure
% hold on
% plot(time, test)
% plot(time, potRR)
% legend('OG', 'RR')
%% FFT
Fs = 1000;
T = 1/Fs;
L = length(time);
t = (0:L-1)*T;

potRL_OG = data.MCU.pots.pot4(:,2); % RL
potRR_OG = data.MCU.pots.pot6(:,2); % RR

potRL_FFT = fft(potRL);
P2 = abs(potRL_FFT/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
figure
hold on
plot(f, P1)

potRR_FFT = fft(potRR);
P2 = abs(potRR_FFT/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
plot(f, P1)
legend('RL', 'RR')


%% 
% dpotRL = diff(potRL)./diff(time);
% dpotRR = diff(potRR)./diff(time);
% 
% dpotRLMask = dpotRL > 0;
% dpotRRMask = dpotRR > 0;
% 
% rear_heaveMask = (dpotRLMask & dpotRRMask) | (~dpotRLMask & ~dpotRRMask);
% rear_heaveMask = [false; rear_heaveMask];
% 
% rear_heaveTime = time(rear_heaveMask);
% [time, rear_heaveMask];
% 
% figure
% hold on
% plot(time(2:end), dpotRL);
% plot(time(2:end), dpotRR);
% 
% % figure
% % hold on
% % plot(rear_heaveTime, potRL(rear_heaveMask))
% % plot(rear_heaveTime, potRR(rear_heaveMask))
% 
% 
% figure
% hold on
% plot(rear_heaveTime, rad2deg(((potRL(rear_heaveMask)-potRR(rear_heaveMask))*MR_rr/track)));
%% Camber Range Calc
warning off
steerFile = 'HT07CamberVsSteerData.xlsx';
steerCamberData = readtable(steerFile);

steer = steerCamberData.Motion_Steering__deg_;
steerCamberFL = steerCamberData.CamberAngle_Left__Front__deg_;
steerCamberFR = steerCamberData.CamberAngle_Right__Front__deg_;
steerCamberRL = steerCamberData.CamberAngle_Left__Rear__deg_;
steerCamberRR = steerCamberData.CamberAngle_Right__Rear__deg_;

ft = fittype('poly2');

steerCamberFitFL = fit(steer, steerCamberFL, ft);
steerCamberFitFR = fit(steer, steerCamberFR, ft);
steerCamberFitRL = fit(steer, steerCamberRL, ft);
steerCamberFitRR = fit(steer, steerCamberRR, ft);

rollFile = 'HT07CamberVsRollData.xlsx';
rollCamberData = readtable(rollFile);

roll = (rollCamberData.Motion_Roll__deg_);
camberFI = rollCamberData.CamberAngle_Left__Front__deg_;
camberFO = rollCamberData.CamberAngle_Right__Front__deg_;
camberRI = rollCamberData.CamberAngle_Left__Rear__deg_;
camberRO = rollCamberData.CamberAngle_Right__Rear__deg_;

% shift to 0 static camber position, steer camber = static camber
camberFI = camberFI - camberFI(roll == 0);
camberFO = camberFO - camberFO(roll == 0);
camberRI = camberRI - camberRI(roll == 0);
camberRO = camberRO - camberRO(roll == 0);

ft = fittype( 'poly2' );

rollCamberFitRL = fit(roll, camberRI, ft);
rollCamberFitRR = fit(roll, camberRO, ft);
rollCamberFitFL = fit(roll, camberFI, ft);
rollCamberFitFR = fit(roll, camberFO, ft);

% steering
% in optimum, positive steer ang = right turn
% steering sensor, positive steer ang = left turn

temp = linspace(0,1,length(steering));
n = length(frontRoAng);
t_ = linspace(0,1,n);

steering = interp1(temp, steering, t_, 'spline')';

steerCamberRangeFL = steerCamberFitFL(-steering);
steerCamberRangeFR = steerCamberFitFR(-steering);
steerCamberRangeRL = steerCamberFitRL(-steering);
steerCamberRangeRR = steerCamberFitRR(-steering);

% roAng
% in optimum, positive roll = right turn, vehicle roll left
 
rollCamberRangeFL = rollCamberFitFL(frontRoAng);
rollCamberRangeFR = rollCamberFitFR(frontRoAng);
rollCamberRangeRL = rollCamberFitRL(rearRoAng);
rollCamberRangeRR = rollCamberFitRR(rearRoAng);

camberFL = steerCamberRangeFL + rollCamberRangeFL;
camberFR = steerCamberRangeFR + rollCamberRangeFR;
camberRL = steerCamberRangeRL + rollCamberRangeRL;
camberRR = steerCamberRangeRR + rollCamberRangeRR;
%% Camber Plot

figure
ax1 = subplot(4,1,1);
hold on
plot(time, frontRoAng)
plot(time, rearRoAng)
title('Roll Angle (deg), Negative - Roll Right (Left Hand Turn)')
legend('Front Roll Angle', 'Rear Roll Angle')

ax2 = subplot(4,1,2);
plot(time, steering)
title('Steering Wheel Angle (deg), Positive - Left Hand Turn')

ax3 = subplot(4,1,3);
hold on
plot(time, camberFL)
plot(time, camberFR)
title('FRONT CAMBER ANGLE (DEG)')
legend('Camber FL', 'Camber FR')

ax4 = subplot(4,1,4);
hold on
plot(time, camberRL)
plot(time, camberRR)
title('REAR CAMBER ANGLE (DEG)')
legend('Camber RL', 'Camber RR')
hold off

linkaxes([ax1, ax2, ax3, ax4], 'x')
%% Lateral Accel and Inverter Comp
fr_Ay = frontLatG*g;
rr_Ay = rearLatG*g;
yawI = yawInertia(Parameters);

fr_Fy = fr_Ay*mass_fr;
rr_Fy = rr_Ay*mass_rr;

fr_moment = fr_Fy*fr2cg;
rr_moment = rr_Fy*rr2cg;

yawMoment = fr_moment-rr_moment;
yawRate = -yawMoment/yawI;

[N, edges] = histcounts(yawRate);
[~, ind] = max(N);
yawRate = yawRate - edges(ind);

yawAng = cumtrapz(time, yawRate);


imu_yawAng = cumtrapz(time, IMU_yawRate);

figure
ax1 = subplot(5,1,1);
hold on
plot(time, rad2deg(yawRate))
plot(time, IMU_yawRate)
% plot(time, (rad2deg(yawRate)+imu_yawRate)./2)
title('Yaw Rate Comparison')
xlabel('Time (s)')
ylabel('Yaw Rate (deg/s)')
legend('Yaw Rate Calc', 'Yaw Rate IMU')

ax2 = subplot(5,1,2);
hold on
plot(time, rad2deg(yawAng))
plot(time, imu_yawAng)
title('Yaw Ang Comparison')
xlabel('Time (s)')
ylabel('Yaw (deg)')
legend('Yaw Calc', 'Yaw IMU')

ax3 = subplot(5,1,3);
hold on
plot(time, frontLatG)
plot(time, rearLatG)
legend('Front LatG', 'Rear LatG')
xlabel('Time (s)')
ylabel('Lat Accel (G)')
title('LatG')

% Inverter
ax4 = subplot(5,1,4);
hold on

plot(time, mc_FL, '.')
plot(time, mc_FR, '.')
plot(time, mc_RL, '.')
plot(time, mc_RR, '.')

title('Wheel Speed')
legend("Front Left","Front Right","Rear Left","Rear Right")
xlabel('Time (s)')
ylabel('Speed (m/s)')

ax5 = subplot(5,1,5);
hold on
plot(time, mc_avg_front)
plot(time, mc_avg_rear)
plot(time, rad2deg(yawRate))
% plot(inverter_time, imu_yawRate)
title('Wheel Speed Average and Yaw Rate Comp')
legend('Front Wheel Speed', 'Rear Wheel Speed', 'Yaw Rate Calc', 'Yaw Rata IMU')
xlabel('Time (s)')
ylabel('Speed (m/s), w (deg/s)')

linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')

%% Tire Fz

% Static Load
Fz_Static = mass.*g;
tot_mass = sum(sum(mass));

% Unsprung lateral load transfer
deltaFzLatUnsprung_fr = uMass_fr*g*frontLatG*hf/track;
deltaFzLatUnsprung_rr = uMass_rr*g*rearLatG*hr/track;

% Geometric lateral load transfer
deltaFzLatGeo_fr = sMass_fr*g*frontLatG*(fr2cg/wb)*hf_rc/track;
deltaFzLatGeo_rr = sMass_rr*g*rearLatG*(rr2cg/wb)*hr_rc/track;

% Sprung lateral load transfer
q = roRate_fr/(roRate_fr+roRate_rr);
deltaFzLatSprung_fr = (sMass_tot*g*frontLatG*cg2RoAxis/track)*q;
deltaFzLatSprung_rr = (sMass_tot*g*rearLatG*cg2RoAxis/track)*(1-q);

% Total lateral load transfer Segers
deltaFzLat_fr = deltaFzLatUnsprung_fr+deltaFzLatGeo_fr+deltaFzLatSprung_fr;
deltaFzLat_rr = deltaFzLatUnsprung_rr+deltaFzLatGeo_rr+deltaFzLatSprung_rr;

% Lateral load transfer Gillespie
deltaFzLat_fr_gil = frontLatG.*(tot_mass.*g./(track))*(cg2RoAxis.*roRate_fr./(roRate_fr+roRate_rr)+cg2RrAxle./wb.*hf_rc);
deltaFzLat_rr_gil = rearLatG.*(tot_mass.*g./(track))*(cg2RoAxis.*roRate_rr./(roRate_fr+roRate_rr)+(wb-cg2RrAxle)./wb.*hr_rc);

% Longitudinal Load Transfer
deltaFzLong = tot_mass.*IMU_longAccel.*cgz./wb;

% Longitudinal Velocity
vel_estimate = (mc_avg_front+mc_avg_rear).*0.5;

% Aero
lift = 0.5.*Cl*rho.*vel_estimate.^2.*A;
drag = 0.5.*Cd*rho.*vel_estimate.^2.*A;

% Longitudinal Aero Load Dist.
AeroFz_long_fr = lift.*(cp2RrAxle./wb)-drag.*(cpz./wb);
AeroFz_long_rr = lift.*((wb-cp2RrAxle)./wb)-drag.*(cpz./wb);

% No Longitudinal Load Dist. (For "Fake" Data)
TireFz_FL_NL = Fz_Static(1)+deltaFzLat_fr_gil+AeroFz_long_fr./2;
TireFz_FR_NL = Fz_Static(3)-deltaFzLat_fr_gil+AeroFz_long_fr./2;
TireFz_RL_NL = Fz_Static(2)+deltaFzLat_rr_gil+AeroFz_long_rr./2;
TireFz_RR_NL = Fz_Static(4)-deltaFzLat_rr_gil+AeroFz_long_rr./2;

% Gillespie
TireFz_FL_gil = Fz_Static(1)-deltaFzLong./2+deltaFzLat_fr_gil+AeroFz_long_fr./2;
TireFz_FR_gil = Fz_Static(3)-deltaFzLong./2-deltaFzLat_fr_gil+AeroFz_long_fr./2;
TireFz_RL_gil = Fz_Static(2)+deltaFzLong./2+deltaFzLat_rr_gil+AeroFz_long_rr./2;
TireFz_RR_gil = Fz_Static(4)+deltaFzLong./2-deltaFzLat_rr_gil+AeroFz_long_rr./2;

% Segers
TireFz_FL = Fz_Static(1)-deltaFzLong./2+deltaFzLat_fr+AeroFz_long_fr./2;
TireFz_FR = Fz_Static(3)-deltaFzLong./2-deltaFzLat_fr+AeroFz_long_fr./2;
TireFz_RL = Fz_Static(2)+deltaFzLong./2+deltaFzLat_rr+AeroFz_long_rr./2;
TireFz_RR = Fz_Static(4)+deltaFzLong./2-deltaFzLat_rr+AeroFz_long_rr./2;

%% Tire Fz Plot
figure
subplot(6,1,1)
hold on
plot(time, deltaFzLat_fr_gil)
plot(time, deltaFzLat_rr_gil)
legend('Front', 'Rear')
title('Lateral Load Transfer, Gillespie')

subplot(6,1,2)
hold on
plot(time, deltaFzLong)
title('Longitudinal Load Transfer')

subplot(6,1,3)
hold on
plot(time, AeroFz_long_fr)
plot(time, AeroFz_long_rr)
legend('Front', 'Rear')
title('Aero Load Distribution')

subplot(6,1,4)
hold on
plot(time, TireFz_FL_gil)
plot(time, TireFz_FR_gil)
plot(time, TireFz_RL_gil)
plot(time, TireFz_RR_gil)
legend('FL', 'FR', 'RL', 'RR')
title('Tire Normal Forces, Gillespie')

subplot(6,1,5)
hold on
plot(time, TireFz_FL)
plot(time, TireFz_FR)
plot(time, TireFz_RL)
plot(time, TireFz_RR)
legend('FL', 'FR', 'RL', 'RR')
title('Tire Normal Forces, Segers')

subplot(6,1,6)
hold on
plot(time, TireFz_FL_NL)
plot(time, TireFz_FR_NL)
plot(time, TireFz_RL_NL)
plot(time, TireFz_RR_NL)
legend('FL', 'FR', 'RL', 'RR')
title('Tire Normal Forces, No Longitudinal Load Dist.')
%% "Fake" Data for Controller

warning off

Beta = deg2rad(12); % Assume Static body slip
IMU_yawRate_rad = deg2rad(IMU_yawRate);

vely_estimate = vel_estimate.*Beta;

steerFile = 'HT07CamberVsSteerData.xlsx';
steerCamberData = readtable(steerFile);
steer = steerCamberData.Motion_Steering__deg_;

wheelSteerData = steerCamberData.SteerAngle_Left__Front__deg_;
wheelSteerFit = fit(steer, wheelSteerData, 'poly2');

wheelSteer = wheelSteerFit(steering)';

% Slip Calc

sa_FL = (vely_estimate+IMU_yawRate_rad.*(wb-b))./(vel_estimate-IMU_yawRate_rad.*track./2)-deg2rad(wheelSteer);
sa_FR = (vely_estimate+IMU_yawRate_rad.*(wb-b))./(vel_estimate+IMU_yawRate_rad.*track./2)-deg2rad(wheelSteer);
sa_RL = (vely_estimate-IMU_yawRate_rad.*b)./(vel_estimate-IMU_yawRate_rad.*track./2);
sa_RR = (vely_estimate-IMU_yawRate_rad.*b)./(vel_estimate+IMU_yawRate_rad.*track./2);

sa_FL_mask = sa_FL < -0.25 | sa_FL > 0.25;
sa_FR_mask = sa_FR < -0.25 | sa_FR > 0.25;
sa_RL_mask = sa_RL < -0.25 | sa_RL > 0.25;
sa_RR_mask = sa_RR < -0.25 | sa_RR > 0.25;

sa_FL(sa_FL_mask) = 0;
sa_FR(sa_FR_mask) = 0;
sa_RL(sa_RL_mask) = 0;
sa_RR(sa_RR_mask) = 0;

sa_FL = smoothdata(sa_FL, 'sgolay');
sa_FR = smoothdata(sa_FR, 'sgolay');
sa_RL = smoothdata(sa_RL, 'sgolay');
sa_RR = smoothdata(sa_RR, 'sgolay');


% sa_RL = - atan((b.*IMU_yawRate_rad + vel_estimate.*sin(Beta))./(-0.5.*track.*IMU_yawRate_rad + vel_estimate.*cos(Beta)));
% sa_RR = - atan((b.*IMU_yawRate_rad + vel_estimate.*sin(Beta))./(0.5.*track.*IMU_yawRate_rad + vel_estimate.*cos(Beta)));
% sa_FL =  - atan((((wb-b).*IMU_yawRate_rad + vel_estimate.*sin(Beta))./((-0.5.*track.*IMU_yawRate_rad + vel_estimate.*cos(Beta)))));
% sa_FR =  - atan((((wb-b).*IMU_yawRate_rad + vel_estimate.*sin(Beta))./((0.5.*track.*IMU_yawRate_rad + vel_estimate.*cos(Beta)))));

figure
hold on
plot(time, rad2deg(sa_FL))
plot(time, rad2deg(sa_FR))
plot(time, rad2deg(sa_RL))
plot(time, rad2deg(sa_RR))
legend('FL', 'FR', 'RL', 'RR')
title('Slip Ang (Deg)')

warning on
%%
% time = data.MCU.pots.pot4(:,1);
% 
% % Unsprung lateral load transfer
% deltaFzLatUnsprung_fr = uMass_fr*g*frontLatG*hf/track;
% deltaFzLatUnsprung_rr = uMass_rr*g*rearLatG*hr/track;
% 
% % Geometric lateral load transfer
% deltaFzLatGeo_fr = sMass_fr*g*frontLatG*(fr2cg/wb)*hf_rc/track;
% deltaFzLatGeo_rr = sMass_rr*g*rearLatG*(rr2cg/wb)*hr_rc/track;
% 
% % Sprung lateral load transfer
% q = roRate_fr/(roRate_fr+roRate_rr);
% deltaFzLatSprung_fr = (sMass_tot*g*frontLatG*cg2RoAxis/track)*q;
% deltaFzLatSprung_rr = (sMass_tot*g*rearLatG*cg2RoAxis/track)*(1-q);
% 
% % Total lateral load transfer
% deltaFzLat_fr = deltaFzLatUnsprung_fr+deltaFzLatGeo_fr+deltaFzLatSprung_fr;
% deltaFzLat_rr = deltaFzLatUnsprung_rr+deltaFzLatGeo_rr+deltaFzLatSprung_rr;
% 
% % Gillespie lateral load transfer
% deltaFzLat_fr = roRate_fr*(mass_tot*cg2RoAxis*frontLatG)/(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis)...
%                     + mass_fr*g*hf_rc*frontLatG;
% deltaFzLat_rr = roRate_rr*(mass_tot*cg2RoAxis*frontLatG)/(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis)...
%                     + mass_rr*g*hr_rc*rearLatG;
% 
% figure
% ax1 = subplot(3,1,1);
% hold on
% plot(time, deltaFzLat_fr)
% plot(time, deltaFzLat_rr)
% plot(time, deltaFzLat_fr)
% plot(time, deltaFzLat_rr)
% title('Total Lateral Load Transfer')
% legend('Total Front Lateral Load Transfer', 'Total Rear Lateral Load Transfer',...
%        'Total Front Lateral Load Transfer Gillespie', 'Total Rear Lateral Load Transfer Gillespie')
% xlabel('Time (s)')
% ylabel('Delta Fz (N)')
% 
% % Tire Loads
% FzTire_static = mass*g;
% FzTireFL = FzTire_static(1)+deltaFzLat_fr;
% FzTireFR = FzTire_static(3)-deltaFzLat_fr;
% FzTireRL = FzTire_static(2)+deltaFzLat_rr;
% FzTireRR = FzTire_static(4)-deltaFzLat_rr;
% 
% ax2 = subplot(3,1,2);
% hold on
% plot(time, FzTireFL)
% plot(time, FzTireFR)
% plot(time, FzTireRL)
% plot(time, FzTireRR)
% title('Tire Normal Forces')
% legend('FL', 'FR', 'RL', 'RR');
% xlabel('Time (s)');
% ylabel('Fz (N)')

loadcell_time = data.MCU.load_cells.FL(:,1);
loadcell_FL = data.MCU.load_cells.FL(:,2)*4.44822;
loadcell_FR = data.MCU.load_cells.FR(:,2)*4.44822;
loadcell_RL = data.MCU.load_cells.RL(:,2)*4.44822;
loadcell_RR = data.MCU.load_cells.RR(:,2)*4.44822;

loadcell_FL = loadcell_FL-mode(loadcell_FL);
loadcell_FR = loadcell_FR-mode(loadcell_FR);
loadcell_RL = loadcell_RL-mode(loadcell_RL);
loadcell_RR = loadcell_RR-mode(loadcell_RR);

deltaLoadcellLat_fr = loadcell_FL-loadcell_FR;
deltaLoadcellLat_rr = loadcell_RL-loadcell_RR;

figure
hold on
plot(loadcell_time, deltaLoadcellLat_fr);
plot(loadcell_time, deltaLoadcellLat_rr);
title('Load Cell Lateral Load Transfer')
legend('Front', 'Rear')

 
%% TPMS Temp Analysis

startingTemp_FL = [];
endingTemp_FL = [];

startingTemp_FR = [];
endingTemp_FR = [];

startingTemp_RL = [];
endingTemp_RL = [];

startingTemp_RR = [];
endingTemp_RR = [];


for i = 16:-1:1
    tempChannel_FL = eval(sprintf('data.TPMS.LF.temp_%d', i));
    mask_FL = tempChannel_FL(:,2) == -100;
    tempChannel_FL(mask_FL,:) = [];
    % tempChannel_FL(1:15, :) = [];

    tempChannel_FL(end-20:end, :) = [];
    startingTemp_FL = [startingTemp_FL, tempChannel_FL(1,2)];
    endingTemp_FL = [endingTemp_FL, tempChannel_FL(end, 2)];

    tempChannel_FR = eval(sprintf('data.TPMS.RF.temp_%d', i));
    mask_FR = tempChannel_FR(:,2) == -100;
    tempChannel_FR(mask_FR,:) = [];
    % tempChannel_FR(1:15, :) = [];

    tempChannel_FR(end-20:end, :) = [];
    startingTemp_FR = [tempChannel_FR(1,2), startingTemp_FR];
    endingTemp_FR = [tempChannel_FR(end, 2), endingTemp_FR];

    tempChannel_RL = eval(sprintf('data.TPMS.LR.temp_%d', i));
    mask_RL = tempChannel_RL(:,2) == -100;
    tempChannel_RL(mask_RL,:) = [];
    % tempChannel_RL(1:15, :) = [];

    tempChannel_RL(end-20:end, :) = [];
    startingTemp_RL = [startingTemp_RL, tempChannel_RL(1,2)];
    endingTemp_RL = [endingTemp_RL, tempChannel_RL(end, 2)];

    tempChannel_RR = eval(sprintf('data.TPMS.RR.temp_%d', i));
    mask_RR = tempChannel_RR(:,2) == -100;
    tempChannel_RR(mask_RR,:) = [];
    % tempChannel_RR(1:15, :) = [];

    tempChannel_RR(end-20:end, :) = [];
    startingTemp_RR = [tempChannel_RR(1,2), startingTemp_RR];
    endingTemp_RR = [tempChannel_RR(end, 2), endingTemp_RR];
end

% Color plot
figure

% FL
minColorLimit = min(startingTemp_FL);
maxColorLimit = max(endingTemp_FL);
colorBound = [minColorLimit, maxColorLimit];

ax1 = subplot(4,2,1);
imagesc(startingTemp_FL)
clim(colorBound)
title('Front Left Starting Temperature')
colorbar

ax2 = subplot(4,2,3);
imagesc(endingTemp_FL)
clim(colorBound)
title('Front Left Ending Temperature')
colorbar

% FR
minColorLimit = min(startingTemp_FR);
maxColorLimit = max(endingTemp_FR);
colorBound = [minColorLimit, maxColorLimit];

subplot(4,2,2)
imagesc(startingTemp_FR)
clim(colorBound)
title('Front Right Starting Temperature')
colorbar

subplot(4,2,4)
imagesc(endingTemp_FR)
clim(colorBound)
title('Front Right Ending Temperature')
colorbar

% RL
minColorLimit = min(startingTemp_RL);
maxColorLimit = max(endingTemp_RL);
colorBound = [minColorLimit, maxColorLimit];

subplot(4,2,5)
imagesc(startingTemp_RL)
clim(colorBound)
title('Rear Left Starting Temperature')
colorbar

subplot(4,2,7)
imagesc(endingTemp_RL)
clim(colorBound)
title('Rear Left Ending Temperature')
colorbar

% RR
minColorLimit = min(startingTemp_RR);
maxColorLimit = max(endingTemp_RR);
colorBound = [minColorLimit, maxColorLimit];

subplot(4,2,6)
imagesc(startingTemp_RR)
clim(colorBound)
title('Rear Right Starting Temperature')
colorbar

subplot(4,2,8)
imagesc(endingTemp_RR)
clim(colorBound)
title('Rear Right Ending Temperature')
colorbar

sgtitle('Tire Temp Analysis')

%% ABS LatG vs Tire Temp

% WIP
