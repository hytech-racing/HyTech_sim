clear;
close all;
clc;

%% Parameters
dt = 0.001;                                     % s, change simulink solver step size too

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

