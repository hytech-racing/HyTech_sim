clear;
close all;
clc;

%% BICYCLE PITCH MODEL

k1 = 29000;     % N/m
k2 = 50500;     % N/m

kt1 = 87563;    % N/m
kt2 = kt1;      % N/m

c1 = 2000;      % N.s/m
c2 = c1;        % N.s/m

a1 = 1.535/2;   % m
a2 = 1.535/2;   % m

m1 = 8;         % kg
m2 = m1;        % kg
m = 83;         % kg

Iy = 235;       % kg*m^2


A = [0                      1                       0                       0                       0                       0                       0                       0;
     (-k1-k2)/m             (-c1-c2)/m              (-k1*a1+k2*a2)/m        (-c1*a1+c2*a2)/m        k1/m                    c1/m                    k2/m                    c2/m;
     0                      0                       0                       1                       0                       0                       0                       0;
     (-a1*k1+a2*k2)/Iy      (-a1*c1+a2*c2)/Iy       (-a1^2*c1-a2^2*k2)/Iy   (-a1^2*c1-a2^2*c2)/Iy   (a1*k1)/Iy              (a1*c1)/Iy              (-a2*k2)/Iy             (-a2*c2)/Iy;
     0                      0                       0                       0                       0                       1                       0                       0;
     k1/m1                  c1/m1                   (a1*k1)/m1              (a1*c1)/m1              (-k1-kt1)/m1            -c1/m1                  0                       0;
     0                      0                       0                       0                       0                       0                       0                       1;
     k2/m2                  c2/m2                   (-a2*k2)/m2             (-a2*c2)/m2             0                       0                       (-k2-kt2)/m2            -c2/m2];

B = [0           0;    
     0           0;
     0           0;
     0           0;
     0           0;
     kt1/m1      0;
     0           0;
     0           kt2/m2];

C = [1  0   0   0   0   0   0   0;
     0  0   1   0   0   0   0   0;
     0  0   0   0   1   0   0   0;
     0  0   0   0   0   0   1   0];


sys = ss(A, B, C, 0);

% step(sys)

%% INPUT TEST DATA

% load('FL Road Interp Michelin AutoX data0053.mat')
% load('FR Road Interp Michelin AutoX data0053.mat')
% load('RL Road Interp Michelin AutoX data0053.mat')
% load('RR Road Interp Michelin AutoX data0053.mat')
% load('Time Road Interp Michelin AutoX data0053.mat')

load('FL Displacement Interp Michelin AutoX data0053.mat')
load('FR Displacement Interp Michelin AutoX data0053.mat')
load('RL Displacement Interp Michelin AutoX data0053.mat')
load('RR Displacement Interp Michelin AutoX data0053.mat')
load('timelsim Michelin AutoX data0053.mat')

% [uniqueTime, ind] = unique(time, 'stable');
% 
% timelsim = uniqueTime(1):0.01:uniqueTime(end);
% FLDispInterp = interp1(uniqueTime, FLDisp(ind), timelsim)';
% FRDispInterp = interp1(uniqueTime, FRDisp(ind), timelsim)';
% RLDispInterp = interp1(uniqueTime, RLDisp(ind), timelsim)';
% RRDispInterp = interp1(uniqueTime, RRDisp(ind), timelsim)';
% timelsim = timelsim';


u = [FLDispInterp./1000 RLDispInterp./1000];

[y, t, x] = lsim(sys, u, timelsim);

figure
tiledlayout(3, 2)
nexttile
hold on
plot(t, y(:, 1) .* 1000)
legend('Sprung Mass Displacement (mm)')

nexttile
hold on
plot(t, x(:, 2) .* 1000)
legend('Sprung Mass Velocity (mm/s)')

nexttile
hold on
plot(t, y(:, 3) .* 1000)
plot(t, y(:, 4) .* 1000)
plot(t, FLDispInterp, '-.')
plot(t, RLDispInterp, '-.')
legend('Front Displacement (mm)', 'Rear Displacement (mm)', 'Front Displacement Input (mm)', 'Rear Displacement Input (mm)')

nexttile
hold on
plot(t, x(:, 6) .* 1000)
plot(t, x(:, 8) .* 1000)
legend('Front Velocity (mm/s)', 'Rear Velocity (mm/s)')

nexttile
hold on
plot(t, rad2deg(y(:, 2)))
legend('Pitch Angle (deg)')

nexttile
hold on
plot(t, rad2deg(x(:, 4)))
legend('Pitch Velocity (deg/s)')