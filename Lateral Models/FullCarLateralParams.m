clear;
close all;
clc;

%% Parameters
dt = 0.0001;                                     % s, change simulink solver step size too

m = 250;                                        % kg, total vehicle mass
Iz_vehicle = 125;                               % kgm^2
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
b = 0.07;                                        % N.m.s/rad, motor damping factor
gearboxEff = 1;                               % Gearbox efficiency
motorOmegaLimit = 2094;                         % rad/s, motor mech. speed limit

% motorOmegaLimit = 1186;

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

warning off
steerParamData = readtable('HT08LeftHandCornerSteerTorqueParams.xlsx');
warning on

driverSteerKin = steerParamData.Motion_Steering__deg_;
wheelSteerKin = steerParamData.SteerAngle_Left__Front__deg_;

% risingSlewRate = 1000;
% fallingSlewRate = -1000;

%%
% load data0053.mat % Michelin AutoX 26s - 70s
% load data0058.mat % Michelin Endurance
load data0028.mat % Michelin Skidpad 
% load data0234.mat % SCC Left Hand Continuous Corner 0 - 30s

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

figure
hold on
plot(steeringTime, steeringData)

steeringData = lowpass(steeringData, 4, 1/dt);
plot(steeringTime, steeringData)
legend('Before', 'After')


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
motorTorqueFLInterp(1:latestInitInd) = [];
motorTorqueFRInterp(1:latestInitInd) = [];
motorTorqueRLInterp(1:latestInitInd) = [];
motorTorqueRRInterp(1:latestInitInd) = [];

wheelSpeedDataFLInterp(1:latestInitInd) = [];
wheelSpeedDataFRInterp(1:latestInitInd) = [];
wheelSpeedDataRLInterp(1:latestInitInd) = [];
wheelSpeedDataRRInterp(1:latestInitInd) = [];

% For HT07 data, negative steer = turning right, change to SAE standard
steeringDataMap = [-130 130];
wheelSteerRange = [23 -23];
wheelSteerDataInput = interp1(steeringDataMap, wheelSteerRange, steeringDataInterp);

wheelSteerDataInput(1:latestInitInd) = [];
steeringDataInterp(1:latestInitInd) = [];
% wheelSteerDataInterpMagnitude = interp1(abs(driverSteerKin), abs(wheelSteerKin), abs(steeringDataInterp));
% wheelSteerDataInterpDir = sign(steeringDataInterp);

% wheelSteerDataInterp = wheelSteerDataInterpMagnitude .* wheelSteerDataInterpDir;


% wheelSteerDataInterp = wheelSteerDataInterp .* -1;

% wheelSteerDataInterp = smoothdata(wheelSteerDataInterp, 'movmedian');




timelsim(1:latestInitInd) = [];
timelsim = 0:dt:dt * length(timelsim) - dt;
timelsim = timelsim';


% maxval = round(timelsim(end));
% 
figure
hold on
plot(timelsim, motorTorqueFLInterp)
plot(timelsim, motorTorqueFRInterp)
plot(timelsim, motorTorqueRLInterp)
plot(timelsim, motorTorqueRRInterp)

tiledlayout(2, 1)
nexttile
hold on
plot(timelsim, -steeringDataInterp)
legend('Driver Steer (Deg)')

nexttile
hold on
plot(timelsim, wheelSteerDataInput)
legend('Wheel Steer (Deg)')
