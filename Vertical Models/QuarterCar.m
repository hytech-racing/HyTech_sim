clear;
close all;
clc;

%% QUARTER CAR

ks = 29000;   % N/m
ku = 87563; % N/m
cs = 2000;  % N.s/m

mu = 8;
ms = 40;

A = [0              1               0               0;
     (-ks-ku)/mu    -cs/mu          ks/mu           cs/mu;
     0              0               0               1;
     ks/ms          cs/ms           -ks/ms          -cs/ms];

B = [0;
     ku/mu;
     0;
     0];

C = [1  0   0   0;
     0  0   1   0];

D = [0;
     0];

sys = ss(A, B, C, D);


step(sys)


[V,D] = eig(A);

wn = sqrt(real(D) .* real(D) + imag(D) .* imag(D));
wn_hz = wn ./ (2 .* pi);
dampingRatio = sqrt(1 - (imag(D) ./ wn) .^ 2);

% [wntest, zeta] = damp(sys);

% figure
% bode(sys2)

%%

load('FL Displacement Interp Michelin AutoX data0053.mat')
load('FR Displacement Interp Michelin AutoX data0053.mat')
load('RL Displacement Interp Michelin AutoX data0053.mat')
load('RR Displacement Interp Michelin AutoX data0053.mat')
load('timelsim Michelin AutoX data0053.mat')

u = FLRoad./1000;

[y, t, x] = lsim(sys, u, timelsim);

figure
tiledlayout(2, 2)
nexttile
hold on
plot(t, x(:, 3) .* 1000)
legend('Sprung Mass Displacement (mm)')

nexttile
hold on
plot(t, x(:, 4) .* 1000)
legend('Sprung Mass Velocity (mm/s)')

nexttile
hold on
plot(t, x(:, 1) .* 1000)
plot(t, FLDispInterp, '-.')
legend('Unsprung Mass Displacement (mm)', 'Unsprung Mass Displacement Input (mm)')

nexttile
hold on
plot(t, x(:, 2) .* 1000)
legend('Unsprung Mass Velocity (mm/s)')

%% test
% testVec = [abs(FLDispInterp) abs(FRDispInterp)];
% FrontDiff = max(testVec, [], 2) - min(testVec, [], 2);
% 
% higherLeft = abs(FLDispInterp) > abs(FRDispInterp);
% FLRoad = FrontDiff;
% FLRoad(~higherLeft) = -1 .* FLRoad(~higherLeft);
% FRRoad = FrontDiff;
% FRRoad(higherLeft)  = -1 .* FRRoad(higherLeft);
% 
% % FLRoad = FrontDiff;
% % FLRoad(FrontDiff < 0) = 0;
% % FRRoad = FrontDiff;
% % FRRoad(FrontDiff > 0) = 0;
% 
% figure
% hold on
% plot(t, FrontDiff)
% plot(t, FLRoad)
% plot(t, FRRoad)


