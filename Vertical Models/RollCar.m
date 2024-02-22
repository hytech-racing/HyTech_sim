clear;
close all;
clc;

%% ROLL CAR

k1 = 29000;     % N/m
k2 = k1;        % N/m

karb = 0;       % N.m/rad
bl = 0.12;         % m

kt1 = 87563;    % N/m
kt2 = kt1;      % N/m

c1 = 3000;      % N.s/m
c2 = c1;        % N.s/m

b1 = 1.2/2;     % m
b2 = 1.2/2;     % m
w = b1 + b2;    % m

m1 = 8;         % kg
m2 = m1;        % kg
m = 83;         % kg

Ix = 75;        % kg*m^2


A = [0                      1                       0                               0                       0                               0                       0                           0;
     (-k1-k2)/m             (-c1-c2)/m              k1/m                            c1/m                    k2/m                            c2/m                    (-k1*b1+k2*b2)/m            (-c1*b1+c2*b2)/m;
     0                      0                       0                               1                       0                               0                       0                           0;
     k1/m1                  c1/m1                   (-k1-kt1)/m1-karb/(m1*bl*w)     -c1/m1                  karb/(m1*bl*w)                  0                       k1*b1/m1+karb/(m1*bl)                    c1*b1/m1;
     0                      0                       0                               0                       0                               1                       0                           0;
     k2/m2                  c2/m2                   karb/(m2*bl*w)                  0                       (-k2-kt2)/m2-karb/(m2*bl*w)     -c2/m2                  (-k2*b2)/m2-karb/(m1*bl)                 (-c2*b2)/m2;
     0                      0                       0                               0                       0                               0                       0                           1;
     (-k1*b1+k2*b2)/Ix      (-c1*b1+c2*b2)/Ix       (k1*b1)/Ix                      (c1*b1)/Ix              (-k2*b2)/Ix                     (-c2*b2)/Ix             (-k1*b1^2-k2*b2^2-karb)/Ix  (-c1*b1^2-c2*b2^2)/Ix];


B = [0          0;
     0          0;
     0          0;
     kt1/m1     0;
     0          0;
     0          kt2/m2;
     0          0;
     0          0];

C = [1  0   0   0   0   0   0   0;
     0  0   1   0   0   0   0   0;
     0  0   0   0   1   0   0   0;
     0  0   0   0   0   0   1   0];

sys = ss(A, B, C, 0);

[y, tOut, x] = step(sys);

%% 
figure

xlim([-1, 1])
ylim([-2, 2])
yline(0)
L = 0.6;

hold on

p1 = plot(0, 0);
p2 = plot(0, 0);
p3 = plot(0, 0);
l1 = plot(0, 0);


for row = 1:size(x, 1)

    delete(p1)
    delete(p2)
    delete(p3)
    delete(l1)
    

    p1 = plot(0,    x(row, 1), 'ko');
    p2 = plot(0.6,  x(row, 3), 'bo');
    p3 = plot(-0.6, x(row, 5), 'ro');

    x1 = x(row, 1) + L*sin(x(row, 7));
    x2 = x(row, 1) - L*sin(x(row, 7));

    l1 = plot([0.6 -0.6], [x1 x2], 'g--');

    
    % drawnow
    pause(0.05)

    
end




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


u = [FLDispInterp./1000 FRDispInterp./1000];
% u = [FLDispInterp./1000 zeros(length(FLDispInterp), 1)];

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
plot(t, y(:, 2) .* 1000)
plot(t, y(:, 3) .* 1000)
plot(t, FLDispInterp, '-.')
plot(t, FRDispInterp, '-.')
legend('Left Displacement (mm)', 'Right Displacement (mm)', 'Left Displacement Input (mm)', 'Right Displacement Input (mm)')

nexttile
hold on
plot(t, x(:, 4) .* 1000)
plot(t, x(:, 6) .* 1000)
legend('Left Velocity (mm/s)', 'Right Velocity (mm/s)')

nexttile
hold on
plot(t, rad2deg(y(:, 4)))
legend('Roll Angle (deg)')

nexttile
hold on
plot(t, rad2deg(x(:, 8)))
legend('Roll Velocity (deg/s)')

%%
figure

xlim([-1, 1])
ylim([-0.015, 0.015])
yline(0)
L = 0.6;

hold on

p1 = plot(0, 0);
p2 = plot(0, 0);
p3 = plot(0, 0);
l1 = plot(0, 0);

for row = 4000:size(x, 1)

    delete(p1)
    delete(p2)
    delete(p3)
    delete(l1)
    

    p1 = plot(0,    x(row, 1), 'ko');
    p2 = plot(0.6,  x(row, 3), 'bo');
    p3 = plot(-0.6, x(row, 5), 'ro');

    x1 = x(row, 1) + L*sin(x(row, 7));
    x2 = x(row, 1) - L*sin(x(row, 7));

    l1 = plot([0.6 -0.6], [x1 x2], 'g--');

    
    % drawnow
    pause(0.01)

    
end

