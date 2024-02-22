clear;
close all;
clc;

%% FULL CAR

chassisH = 0.315;       % m
wHubH = 0.2;            % m

% chassisH = 0;         % m
% wHubH = 0;            % m

k1 = 28000;     % N/m
k2 = k1;        % N/m

k3 = 50000;     % N/m
k4 = k3;        % N/m

karbf = 0;          % N.m/rad
blf = 0.12;         % m
karbf = karbf/blf;  % N/rad

karbr = 0;          % N.m/rad
blr = 0.12;         % m
karbr = karbr/blr;  % N/rad

kt1 = 87563;    % N/m
kt2 = kt1;      % N/m
kt3 = kt1;      % N/m
kt4 = kt1;      % N/m

c1 = 2500;      % N.s/m
c2 = c1;        % N.s/m
c3 = 2500;      % N.s/m
c4 = c3;        % N.s/m

a1 = 1.535/2;   % m
a2 = 1.535/2;   % m

b1 = 0.6;     % m
b2 = 0.6;     % m

wf = b1 + b2;   % m
wr = b1 + b2;   % m

m1 = 8;         % kg
m2 = m1;        % kg
m3 = m1;        % kg
m4 = m1;        % kg

m = 250;        % kg

Ix = 75;        % kg*m^2
Iy = 235;       % kg*m^2



A = [0                                  1                                   0                               0               0                               0               0                               0               0                               0               0                                                       0                                               0                                               0;
     (-k1-k2-k3-k4)/m                   (-c1-c2-c3-c4)/m                    k1/m                            c1/m            k2/m                            c2/m            k3/m                            c3/m            k4/m                            c4/m            (-k1*b1+k2*b2-k3*b1+k4*b2)/m                            (-c1*b1+c2*b2-c3*b1+c4*b2)/m                    (-k1*a1-k2*a1+k3*a2+k4*a2)/m                    (-c1*a1-c2*a1+c3*a2+c4*a2)/m;
     0                                  0                                   0                               1               0                               0               0                               0               0                               0               0                                                       0                                               0                                               0;
     k1/m1                              c1/m1                               (-k1-kt1)/m1-karbf/(wf*m1)      -c1/m1          karbf/(wf*m1)                   0               0                               0               0                               0               (k1*b1+karbf)/m1                                        (c1*b1)/m1                                      (k1*a1)/m1                                      (c1*a1)/m1;
     0                                  0                                   0                               0               0                               1               0                               0               0                               0               0                                                       0                                               0                                               0;
     k2/m2                              c2/m2                               karbf/(wf*m2)                   0               (-k2-kt2)/m2-karbf/(wf*m2)      -c2/m2          0                               0               0                               0               (-k2*b2-karbf)/m2                                       (-c2*b2)/m2                                     (k2*a1)/m2                                      (c2*a1)/m2;
     0                                  0                                   0                               0               0                               0               0                               1               0                               0               0                                                       0                                               0                                               0;
     k3/m3                              c3/m3                               0                               0               0                               0               (-k3-kt3)/m3-karbr/(wr*m3)      -c3/m3          karbr/(wr*m3)                   0               (k3*b1+karbr)/m3                                        (c3*b1)/m3                                      (-k3*a2)/m3                                     (-c3*a2)/m3;
     0                                  0                                   0                               0               0                               0               0                               0               0                               1               0                                                       0                                               0                                               0;
     k4/m4                              c4/m4                               0                               0               0                               0               karbr/(wr*m4)                   0               (-k4-kt4)/m4-karbr/(wr*m4)      -c4/m4          (-k4*b2-karbr)/m4                                       (-c4*b2)/m4                                     (-k4*a2)/m4                                     (-c4*a2)/m4;
     0                                  0                                   0                               0               0                               0               0                               0               0                               0               0                                                       1                                               0                                               0;
     (-b1*k1+b2*k2-b1*k3+b2*k4)/Ix      (-b1*c1+b2*c2-b1*c3+b2*c4)/Ix       (b1*k1)/Ix+karbf/(wf*Ix)        (b1*c1)/Ix      (-b2*k2)/Ix-karbf/(wf*Ix)       (-b2*c2)/Ix     (b1*k3)/Ix+karbr/(wf*Ix)        (b1*c3)/Ix      (-b2*k4)/Ix-karbr/(wr*Ix)       (-b2*c4)/Ix     (-b1^2*k1-b2^2*k2-b1^2*k3-b2^2*k4-karbf-karbr)/Ix       (-b1^2*c1-b2^2*c2-b1^2*c3-b2^2*c4)/Ix           (-b1*a1*k1+b2*a1*k2+b1*a2*k3-b2*a2*k4)/Ix       (-b1*a1*c1+b2*a1*c2+b1*a2*c3-b2*a2*c4)/Ix;
     0                                  0                                   0                               0               0                               0               0                               0               0                               0               0                                                       0                                               0                                               1;
     (-a1*k1-a1*k2+a2*k3+a2*k4)/Iy      (-a1*c1-a1*c2+a2*c3+a2*c4)/Iy       (a1*k1)/Iy                      (a1*c1)/Iy      (a1*k2)/Iy                      (a1*c2)/Iy      (-a2*k3)/Iy                     (-a2*c3)/Iy     (-a2*k4)/Iy                     (-a2*c4)/Iy     (-a1*b1*k1+a1*b2*k2+a2*b1*k3-a2*b2*k4)/Iy               (-a1*b1*c1+a1*b2*c2+a2*b1*c3-a2*b2*c4)/Iy       (-a1^2*k1-a1^2*k2-a2^2*k3-a2^2*k4)/Iy           (-a1^2*c1-a1^2*c2-a2^2*c3-a2^2*c4)/Iy];


B = [0          0           0           0;
     0          0           0           0;
     0          0           0           0;
     kt1/m1     0           0           0;
     0          0           0           0;
     0          kt2/m2      0           0;
     0          0           0           0;
     0          0           kt3/m3      0;
     0          0           0           0;
     0          0           0           kt4/m4;
     0          0           0           0;
     0          0           0           0;
     0          0           0           0;
     0          0           0           0];

C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0 0 0;     
     0 0 0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0 1 0];

sys = ss(A, B, C, 0);

[y, t, x] = step(sys);
h = stepplot(sys);

% setoptions(h, 'OutputLabels', {'sMass'})

[V, D] = eig(A);
isstable(sys);

[b, a] = ss2tf(A, B, C, zeros(7, 4), 1);

%% ANIMATE DISPLACEMENT
figure

imp = 2;

xlim([-2,   0.5])
ylim([-1,   1])
zlim([-0.5, 1.5])

grid on
box on

set(gcf, 'Position',  [100, 100, 1100, 800])
set(gca, 'YDir', 'reverse')

xlabel('X')
ylabel('Y')
zlabel('Z')

view([35 35])

halfTrack = 0.6;
halfWB = 1.535/2;

hold on

p1 = plot3(0, 0, 0);
p2 = plot3(0, 0, 0);
p3 = plot3(0, 0, 0);
p4 = plot3(0, 0, 0);
p5 = plot3(0, 0, 0);

pl1 = plot3(0, 0, 0);
pl2 = plot3(0, 0, 0);

l1 = plot3(0, 0, 0);
l2 = plot3(0, 0, 0);

an = annotation('textbox', 'String', 't =');

for row = 1:2:size(x, 1)

    delete(p1)
    delete(p2)
    delete(p3)
    delete(p4)
    delete(p5)

    delete(pl1)
    delete(pl2)

    delete(l1)
    delete(l2)

    delete(an)

    p1 = plot3(0,            -0.6,    x(row, 3, imp), 'b*');
    p2 = plot3(0,            0.6,     x(row, 5, imp), 'r*');
    p3 = plot3(-halfWB*2,    -0.6,    x(row, 7, imp), 'b^');
    p4 = plot3(-halfWB*2,    0.6,     x(row, 9, imp), 'r^');

    pl1 = plot3([0 0],  [-0.6 0.6], [x(row, 3, imp) x(row, 5, imp)], 'b-');
    pl2 = plot3([-halfWB*2 -halfWB*2], [-0.6 0.6], [x(row, 7, imp) x(row, 9, imp)], 'r-');
    
    p5 = plot3(-halfWB,       0,      x(row, 1, imp), 'go');

    leftHalf  = x(row, 1, imp) + halfTrack*sin(x(row, 11, imp));
    rightHalf = x(row, 1, imp) - halfTrack*sin(x(row, 11, imp));

    frontHalf = x(row, 1, imp) + halfWB*sin(x(row, 13, imp));
    rearHalf  = x(row, 1, imp) - halfWB*sin(x(row, 13, imp));

    l1 = plot3([-halfWB -halfWB], [0.6 -0.6], [rightHalf leftHalf], 'g--');
    l2 = plot3([0, -halfWB*2],    [0,  0],    [frontHalf rearHalf], 'b--');

    legend('FL', 'FR', 'RL', 'RR', 'FRONT AXLE', 'REAR AXLE', 'CENTER', 'ROLL', 'PITCH')
    
    an = annotation('textbox', [.7, .8, .1, .1], 'String', sprintf('t = %.2f s', t(row)));
    
    drawnow limitrate

    
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

% u = [FLRoadInterp./1000 FRRoadInterp./1000 RLRoadInterp./1000 RRRoadInterp./1000];

u = [FLDispInterp./1000 FRDispInterp./1000 RLDispInterp./1000 RRDispInterp./1000];

[y, t, x] = lsim(sys, u, timelsim);

figure
tiledlayout(5, 2)
nexttile
hold on
plot(t, x(:, 1) .* 1000)
legend('Sprung Mass Displacement (mm)')

nexttile
hold on
plot(t, x(:, 2) .* 1000)
legend('Sprung Mass Velocity (mm/s)')

nexttile
hold on
plot(t, x(:, 3) .* 1000, 'b-')
plot(t, x(:, 5) .* 1000, 'r-')
plot(t, FLDispInterp, 'b-.')
plot(t, FRDispInterp, 'r-.')
legend('FL Displacement (mm)', 'FR Displacement (mm)', 'FL Displacement Input (mm)', 'FR Displacement Input (mm)')

nexttile
hold on
plot(t, x(:, 4) .* 1000)
plot(t, x(:, 6) .* 1000)
legend('FL Velocity (mm/s)', 'FR Velocity (mm/s)')

nexttile
hold on
plot(t, x(:, 7) .* 1000, 'b-')
plot(t, x(:, 9) .* 1000, 'r-')
plot(t, RLDispInterp, 'b-.')
plot(t, RRDispInterp, 'r-.')
legend('RL Displacement (mm)', 'RR Displacement (mm)', 'RL Displacement Input (mm)', 'RR Displacement Input (mm)')

nexttile
hold on
plot(t, x(:, 8) .* 1000)
plot(t, x(:, 10) .* 1000)
legend('RL Velocity (mm/s)', 'RR Velocity (mm/s)')

nexttile
hold on
plot(t, rad2deg(x(:, 11)))
legend('Roll Angle (deg)')

nexttile
hold on
plot(t, rad2deg(x(:, 12)))
legend('Roll Velocity (deg/s)')

nexttile
hold on
plot(t, rad2deg(x(:, 13)))
legend('Pitch Angle (deg)')

nexttile
hold on
plot(t, rad2deg(x(:, 14)))
legend('Pitch Velocity (deg/s)')

%% ANIMATE DISPLACEMENT

figure

title('Displacement Visualizer')

fps = 20;

xlim([-2,   0.5])
ylim([-1,   1])
zlim([wHubH-0.025, chassisH+0.025])

grid on
box on

set(gcf, 'Position',  [100, 100, 1100, 800])
set(gca, 'YDir', 'reverse')

xlabel('X')
ylabel('Y')
zlabel('Z')

view([35 35])

halfTrack = 0.6;
halfWB = 1.535/2;

hold on

p1 = plot3(0,            -0.6,    0, 'b*');
p2 = plot3(0,            0.6,     0, 'r*');
p3 = plot3(-halfWB*2,    -0.6,    0, 'b^');
p4 = plot3(-halfWB*2,    0.6,     0, 'r^');
p5 = plot3(-halfWB,       0,      0, 'go', 'LineWidth', 2);
p6 = plot3(-halfWB,       0,      0, 'go', 'LineWidth', 2);

pl1 = plot3([0 0],  [-0.6 0.6], [0 0], 'b-');
pl2 = plot3([-halfWB*2 -halfWB*2], [-0.6 0.6], [0 0], 'r-');

l1 = plot3([-halfWB -halfWB], [0.6 -0.6], [0 0], 'g-', 'LineWidth', 2);
l2 = plot3([0, -halfWB*2],    [0,  0],    [0 0], 'b-', 'LineWidth', 2);
l3 = plot3([-halfWB -halfWB], [0.6 -0.6], [0 0], 'g--', 'LineWidth', 2);
l4 = plot3([0, -halfWB*2],    [0,  0],    [0 0], 'b--', 'LineWidth', 2);

an = annotation('textbox', [.7, .8, .1, .1], 'String', 't = ');

tic
for row = 1500:4:size(x, 1)

    set(p1, 'ZData', x(row, 3) + wHubH);
    set(p2, 'ZData', x(row, 5) + wHubH);
    set(p3, 'ZData', x(row, 7) + wHubH);
    set(p4, 'ZData', x(row, 9) + wHubH);

    set(pl1, 'ZData', [x(row, 3) + wHubH x(row, 5) + wHubH]);
    set(pl2, 'ZData', [x(row, 7) + wHubH x(row, 9) + wHubH]);
    
    set(p5, 'ZData', x(row, 1) + chassisH);

    leftHalf  = x(row, 1) + halfTrack*sin(x(row, 11));
    rightHalf = x(row, 1) - halfTrack*sin(x(row, 11));

    frontHalf = x(row, 1) + halfWB*sin(x(row, 13));
    rearHalf  = x(row, 1) - halfWB*sin(x(row, 13));

    set(l1, 'ZData', [rightHalf + chassisH leftHalf + chassisH]);
    set(l2, 'ZData', [frontHalf + chassisH rearHalf + chassisH])

    set(p6, 'ZData', x(row, 1)  + wHubH);

    set(l3, 'ZData', [rightHalf + wHubH leftHalf + wHubH]);
    set(l4, 'ZData', [frontHalf + wHubH rearHalf + wHubH]);

    legend('FL', 'FR', 'RL', 'RR', 'FRONT AXLE', 'REAR AXLE', 'CENTER - C', 'ROLL - C', 'PITCH - C',  'CENTER - W', 'ROLL - W', 'PITCH - W')
    
    timeStr = sprintf('t = %.2f s', t(row));
    set(an, 'String', timeStr)

    if toc > (1/fps)
        drawnow limitrate;
        tic;
    end
    
    
end
