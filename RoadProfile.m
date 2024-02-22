clear;
close all;
clc;


%% Vehicle Param Estimate (STATIC)
m2mm = 1000;
mm2m = 1/m2mm;

HT07_vehicle_parameters;
MR_fr = 0.938;
MR_rr = 0.84;
track = 1.2; % m
cgz = 0.2;
roRate_fr = 15700; % Nm/rad
roRate_rr = 35000; % Nm/rad

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
IMU_longAccel = smoothdata(movmean(IMU_longAccel, 30), 'sgolay');

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

% figure
% tiledlayout(3,1)
% nexttile
% hold on
% plot(time, potFL);
% plot(time, potFR);
% plot(time, potRL);
% plot(time, potRR);
% legend('FL OG', 'FR OG', 'RL OG', 'RR OG')

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

% nexttile
% hold on
% plot(time, potFL);
% plot(time, potFR);
% plot(time, potRL);
% plot(time, potRR);
% legend('FL OG (mm)', 'FR OG (mm)', 'RL OG (mm)', 'RR OG (mm)')

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

% nexttile
% hold on
% plot(time, potFL);
% plot(time, potFR);
% plot(time, potRL);
% plot(time, potRR);
% legend('FL SHIFTED (mm)', 'FR SHIFTED (mm)', 'RL SHIFTED (mm)', 'RR SHIFTED (mm)')

potFL = filloutliers(potFL, 'nearest', 'mean');
potFR = filloutliers(potFR, 'nearest', 'mean');
potRL = filloutliers(potRL, 'nearest', 'mean');
potRR = filloutliers(potRR, 'nearest', 'mean');

potFL(potFL < 0.7 & potFL > -0.7) = 0;
potFR(potFR < 0.7 & potFR > -0.7) = 0;
potRL(potRL < 0.7 & potRL > -0.7) = 0;
potRR(potRR < 0.7 & potRR > -0.7) = 0;

potFLProcessed = smoothdata((potFL), 'sgolay');
potFRProcessed = smoothdata((potFR), 'sgolay');
potRLProcessed = smoothdata((potRL), 'sgolay');
potRRProcessed = smoothdata((potRR), 'sgolay');



%% Final Processed Pot Plots
figure
tiledlayout(4,1);
tile1 = nexttile;
hold on
plot(time, potFLProcessed, '-.');
plot(time, potFRProcessed, '-.');
plot(time, potRLProcessed, '.-');
plot(time, potRRProcessed, '.-');
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

frontRoAng = atand((((potFLProcessed)-(potFRProcessed))*MR_fr/(track*1000)));
rearRoAng  = atand((((potRLProcessed)-(potRRProcessed))*MR_rr/(track*1000)));

leftPitchAng = abs(atand((((potFLProcessed)*MR_fr-(potRLProcessed)*MR_rr)/(track*1000)))) .* sign(IMU_longG);
rightPitchAng  = abs(atand((((potFRProcessed)*MR_fr-(potRRProcessed)*MR_rr)/(track*1000)))) .* sign(IMU_longG);

frontLatG = -(deg2rad(frontRoAng)*(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis))/(mass_tot*g*cg2RoAxis);
rearLatG = -(deg2rad(rearRoAng)*(roRate_fr+roRate_rr-mass_tot*g*cg2RoAxis))/(mass_tot*g*cg2RoAxis);

tile3 = nexttile;
hold on
plot(time, frontRoAng)
plot(time, frontLatG)
plot(time, rearRoAng)
plot(time, rearLatG)
plot(data.IMU.lat_accel(:,1),-movmean(data.IMU.lat_accel(:,2)./g, 5))
legend('Front Roll Angle', 'Front LatG Calc', 'Rear Roll Angle', 'Rear LatG Calc', 'LatG IMU', 'AutoUpdate', 'off')

tile4 = nexttile;
hold on
plot(time, leftPitchAng)
plot(time, rightPitchAng)
plot(time, IMU_longG)
legend('Left Pitch Angle', 'Right Pitch Angle', 'LongG IMU')

linkaxes([tile1, tile2, tile3, tile4], 'x')
zoom xon

%% 

FLDisp = potFLProcessed .* MR_fr;
FRDisp = potFRProcessed .* MR_fr;
RLDisp = potRLProcessed .* MR_rr;
RRDisp = potRRProcessed .* MR_rr;

% figure
% tiledlayout(2,1);
% tile1 = nexttile;
% hold on
% plot(time, potFLProcessed, '-.');
% plot(time, potFRProcessed, '-.');
% plot(time, FLDisp, '.-')
% plot(time, FRDisp, '.-')
% legend('FL POT', 'FR POT', 'FL WHEEL', 'FR WHEEL')
% 
% tile2 = nexttile;
% hold on
% plot(time, potRLProcessed, '-.');
% plot(time, potRRProcessed, '-.');
% plot(time, RLDisp, '.-')
% plot(time, RRDisp, '.-')
% legend('RL POT', 'RR POT', 'RL WHEEL', 'RR WHEEL')



fronth = tand(frontRoAng) .* (track ./ 2 .* m2mm);
rearh  = tand(rearRoAng)  .* (track ./ 2 .* m2mm);
% avgh = (fronth + rearh)./2;

FrontDispSum = abs(FLDisp) + abs(FRDisp);
FLRoad = (FrontDispSum ./ 2) - abs(FLDisp);
FRRoad = (FrontDispSum ./ 2) - abs(FRDisp);

RearDispSum = abs(RLDisp) + abs(RRDisp);
RLRoad = (RearDispSum ./ 2) - abs(RLDisp);
RRRoad = (RearDispSum ./ 2) - abs(RRDisp);


% fronth = 0;
% rearh  = 0;

% FLDispNoRoll = FLDisp - fronth;
% FRDispNoRoll = FRDisp + fronth;
% RLDispNoRoll = RLDisp - rearh;
% RRDispNoRoll = RRDisp + rearh;

% figure
% hold on
% plot(time, FLDispNoRoll)
% % plot(time, FRDispNoRoll)
% plot(time, RLDispNoRoll)
% % plot(time, RRDispNoRoll)
% legend('FLDispNoRoll', 'RLDispNoRoll')

lefth = tand(leftPitchAng) .* (wb ./ 2 .* m2mm);
righth = tand(rightPitchAng) .* (wb ./ 2 .* m2mm);

FLRoad = FLRoad - lefth;
FRRoad = FRRoad - righth;
RLRoad = RLRoad + lefth;
RRRoad = RRRoad + righth;

FLRoad = abs(FLRoad) .* sign(FLDisp);
FRRoad = abs(FRRoad) .* sign(FRDisp);
RLRoad = abs(RLRoad) .* sign(RLDisp);
RRRoad = abs(RRRoad) .* sign(RRDisp);


% FLRoad = FLDisp;
% FRRoad = FRDisp;
% RLRoad = RLDisp;
% RRRoad = RRDisp;

FLRoad = filloutliers(FLRoad, 'nearest', 'mean');
FRRoad = filloutliers(FRRoad, 'nearest', 'mean');
RLRoad = filloutliers(RLRoad, 'nearest', 'mean');
RRRoad = filloutliers(RRRoad, 'nearest', 'mean');

figure
hold on
plot(time, FLRoad)
plot(time, FRRoad)
plot(time, RLRoad)
plot(time, RRRoad)
legend('FLRoad', 'FRRoad', 'RLRoad', 'RRRoad')
%% 

figure
xlim([-10, 10])
ylim([-15, 15])
yline(0)
startInd = find(time > 85, 1);
hold on
for t = startInd:length(time)

    p1 = plot(-10, FLDisp(t), 'bo');
    p2 = plot(10,  FRDisp(t), 'bo');
    p3 = plot(-10, RLDisp(t), 'ro');
    p4 = plot(10,  RRDisp(t), 'ro');

    legend('FL', 'FR', 'RL', 'RR')
    an = annotation('textbox', 'String', sprintf('t = %.2f', time(t)));
    
    drawnow
    
    delete(p1)
    delete(p2)
    delete(p3)
    delete(p4)
    delete(an)

    
end

%% 
