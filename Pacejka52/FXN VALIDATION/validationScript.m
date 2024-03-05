%% getTireForces.m FXN VALIDATION (PACEJKA MODEL)

%% PURE

clear;
clc;
close all;

% HOOSIER 16x7.5-10 LC0 - FY, MX, MZ - 8PSI
load B1965run16.mat
kPa2psi = 0.145038;
P = P*kPa2psi;

idx     = ET > 130 & ET < 650;
EText   = ET(idx);
SAext   = SA(idx);
FZext   = FZ(idx);
FYext   = FY(idx);
MZext   = MZ(idx);
MXext   = MX(idx);
RLext   = RL(idx);
IAext   = IA(idx);
SLext   = SL(idx);
Pext    = P(idx);

% Fit a spline using all points and find the zeros
m = 1:length(SAext);
sp = spline(m, SAext);
z = fnzeros(sp);
z = round(z(1, :));
z = [1, z, length(SAext)]; % hardcode in the first and last zeros

row = 0;

% Splits up data per +/-12 deg SA segment
for n = 1:2:length(z) - 1
    sa = SAext(z(n) : z(n + 2));
    fz = FZext(z(n) : z(n + 2));
    fy = FYext(z(n) : z(n + 2));
    mz = MZext(z(n) : z(n + 2));
    mx = MXext(z(n) : z(n + 2));
    rl = RLext(z(n) : z(n + 2));
    ia = IAext(z(n) : z(n + 2));
    sl = SLext(z(n) : z(n + 2));

    fy = mean(fz) .* fy ./ fz;
    mz = mean(fz) .* mz ./ fz;
    mx = mean(fz) .* mx ./ fz;
    fz = mean(fz) .* ones(length(fz), 1);

    sp_fy = csaps(sa, fy, 0.1);
    sp_mz = csaps(sa, mz, 0.1);
    sp_mx = csaps(sa, mx, 0.1);
    sp_rl = csaps(sa, rl, 0.1);

    for slipAng = floor(min(sa)) : 0.25 : ceil(max(sa))

        row = row + 1;
        fmdata(row, 1) = slipAng;
        fmdata(row, 2) = round(mean(ia));
        fmdata(row, 3) = mean(fz);
        fmdata(row, 4) = fnval(sp_fy, slipAng);
        fmdata(row, 5) = fnval(sp_mz, slipAng);
        fmdata(row, 6) = fnval(sp_mx, slipAng);
        fmdata(row, 7) = mean(sl);

    end
end

% Sorts rows based on IA -> SA -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);

ext.SA = fmdata(:,1);
ext.IA = fmdata(:,2);
ext.FZ = fmdata(:,3);
ext.FY = fmdata(:,4);
ext.MZ = fmdata(:,5);
ext.MX = fmdata(:,6);
ext.SL = fmdata(:,7);

% GET FORCES FROM FUNCTION

INPUT = [ext.SL ext.SA ext.FZ ext.IA];
tireForces = getTireForces(INPUT);

% INPUTY COMPARISON

% close all;

tiledlayout(1, 3)
sgtitle('PURE SLIP FXN. VALIDATION')

% FY COMPARISON
ax1 = nexttile;
hold on
grid on
plot3(INPUT(:, 2),INPUT(:, 3), ext.FY, 'k.')
plot3(INPUT(:, 2),INPUT(:, 3), tireForces.PURE.FY, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load (N)')
zlabel('Lateral Force [N]')
legend('Data Pts','FITTED DATA')
title('FY COMPARISON AT 8 PSI')

% MZ COMPARISON
ax2 = nexttile;
hold on
grid on
plot3(INPUT(:, 2),INPUT(:, 3), ext.MZ, 'k.')
plot3(INPUT(:, 2),INPUT(:, 3), tireForces.PURE.MZ, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load (N)')
zlabel('Aligning Torque [N-m]')
legend('Data Pts','FITTED DATA')
title('MZ COMPARISON AT 8 PSI')



% HOOSIER 6/18.0-10 LC0 - FX - 12PSI
clear;
clc;
load B1654run36.mat

kPa2psi = 0.145038;
P = P*kPa2psi;

idx     = ET > 25 & ET < 208;
EText   = ET(idx);
SAext   = SA(idx);
FZext   = FZ(idx);
FYext   = FY(idx);
FXext   = FX(idx);
MZext   = MZ(idx);
MXext   = MX(idx);
SLext   = SL(idx);
SRext   = SR(idx);
RLext   = RL(idx);
IAext   = IA(idx);
Pext    = P(idx);

m = 1:length(SLext);
sp = spline(m, SLext);
z = ischange(SLext, 'linear');
z = [1 m(z) length(SLext)];

ptTol = 30;

% New data structure row counter
row = 0;

clear fmdata
clear ext

% Splits up data per SL segment
for n = 1:2:length(z) - 1

    sa = SAext(z(n) + ptTol : z(n + 2) - ptTol);
    fz = FZext(z(n) + ptTol : z(n + 2) - ptTol);
    fy = FYext(z(n) + ptTol : z(n + 2) - ptTol);
    mz = MZext(z(n) + ptTol : z(n + 2) - ptTol);
    mx = MXext(z(n) + ptTol : z(n + 2) - ptTol);
    rl = RLext(z(n) + ptTol : z(n + 2) - ptTol);
    ia = IAext(z(n) + ptTol : z(n + 2) - ptTol);
    sl = SLext(z(n) + ptTol : z(n + 2) - ptTol);
    sr = SRext(z(n) + ptTol : z(n + 2) - ptTol);
    fx = FXext(z(n) + ptTol : z(n + 2) - ptTol);

    fx = smoothdata(fx, 'sgolay');

    % tiledlayout(3, 1)
    % nexttile
    % plot(fz)
    % nexttile
    % plot(sl)
    % nexttile
    % plot(fx)

    fx = abs(mean(fz)) .* fx ./ abs(fz);
    fz = mean(fz) .* ones(length(fz), 1);
    
    sp_fx = csaps(sl, fx);

    for slipRatio = -0.115 : 0.005 : 0.115

        row = row + 1;
        fmdata(row, 1) = slipRatio;
        fmdata(row, 2) = round(mean(ia));
        fmdata(row, 3) = mean(fz);
        fmdata(row, 4) = fnval(sp_fx, slipRatio);
        fmdata(row, 5) = mean(sa);

    end
end

% Sorts rows based on IA -> SL -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);

ext.SL = fmdata(:,1);
ext.IA = fmdata(:,2);
ext.FZ = fmdata(:,3);
ext.FX = fmdata(:,4);
ext.SA = fmdata(:,5);

% GET FORCES FROM FUNCTION

INPUT = [ext.SL ext.SA ext.FZ ext.IA];
tireForces = getTireForces(INPUT);

% INPUTX COMPARISON

nexttile

% FX COMPARISON
hold on
grid on
plot3(INPUT(:, 1),INPUT(:, 3), ext.FX, 'k.')
plot3(INPUT(:, 1),INPUT(:, 3), tireForces.PURE.FX, 'ro')
xlabel('Longitudinal Slip')
ylabel('Vertical Load (N)')
zlabel('Longitudinal Force [N]')
legend('Data Pts','FITTED DATA')
title('FX COMPARISON AT 12 PSI')

%% ALL GOOD? 

%{

      ╱|、
    (˚ˎ 。7  
     |、˜〵          
     じしˍ,)ノ

%}

%% COMBINED CHECK 
close all

% TESTSL = linspace(-0.12, 0.12, 100)';
TESTSA = linspace(-6,    6,   100)';
% TESTFZ = -linspace(700, 0, 100)';

TESTIA = linspace(0,     0,    100)';

figure
axis square
for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-1000, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    
    plot(tireForces.COMBINED.FY, tireForces.COMBINED.FX, 'ro')
    % plot3(tireForces.COMBINED.FY ,tireForces.COMBINED.FX, TESTSL, 'r')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'r')
    % legend('1000N')
    xlabel('FY')
    ylabel('FX')
    drawnow

end

for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-700, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    
    plot(tireForces.COMBINED.FY, tireForces.COMBINED.FX, 'go')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'g')
    % legend('700N')
    xlabel('FY')
    ylabel('FX')
    drawnow

end

for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-500, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on

    plot(tireForces.COMBINED.FY, tireForces.COMBINED.FX, 'bo')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'b')
    % legend('500N')
    xlabel('FY [N]')
    ylabel('FX [N]')
    drawnow limitrate

end

title('Friction Ellipse')
annotation('textbox', [0.2, 0.1, 0.1, 0.1] , 'String',  '1000N --> 700N --> 500N', 'FitBoxToText','on')
%% 

figure
for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-1000, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    plot3(TESTSL, TESTSA, tireForces.COMBINED.FX, 'ro')
    plot3(TESTSL, zeros(size(TESTSL)), tireForces.PURE.FX, 'b*')

    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'g')
    % legend('700N')
    xlabel('SL')
    ylabel('SA')
    zlabel('FX')
    drawnow

end
%% 

figure
for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-1000, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    plot3(TESTSL, TESTSA, tireForces.COMBINED.FY, 'ro')
    plot3(zeros(size(TESTSA)), TESTSA, tireForces.PURE.FY, 'b*')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'g')
    % legend('700N')
    xlabel('SL')
    ylabel('SA')
    zlabel('FY')
    drawnow limitrate

end

%% COMBINED CHECK 
% close all

% TESTSL = linspace(-0.12, 0.12, 100)';
TESTSA = linspace(-12,    12,   100)';
% TESTFZ = -linspace(700, 0, 100)';

TESTIA = linspace(0,     0,    100)';

figure
axis square
% axis square
for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-1000, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    plot(tireForces.COMBINED.FY, tireForces.COMBINED.FX, 'ro')
    % plot3(tireForces.COMBINED.FY ,tireForces.COMBINED.FX, TESTSL, 'r')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'r')
    % legend('1000N')
    xlabel('FY')
    ylabel('FX')
    drawnow limitrate

end

for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-700, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    plot(tireForces.COMBINED.FY, tireForces.COMBINED.FX, 'go')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'g')
    % legend('700N')
    xlabel('FY')
    ylabel('FX')
    drawnow limitrate

end

for slINC = -0.15:0.005:0.15
    TESTFZ = repelem(-500, 100)';

    TESTSL = repelem(slINC, 100)';
    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);
    % hold on
    % box, grid on
    % plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
    % legend('COMBINED FX')
    % xlabel('SL')
    % ylabel('FZ')
    % zlabel('FX')
    hold on
    grid on
    plot(tireForces.COMBINED.FY, tireForces.COMBINED.FX, 'bo')
    % plot(tireForces.PURE.FY, tireForces.PURE.FX, 'b')
    % legend('500N')
    xlabel('FY [N]')
    ylabel('FX [N]')
    drawnow limitrate

end

title('Friction Ellipse')
annotation('textbox', [0.2, 0.1, 0.1, 0.1] , 'String',  '1000N --> 700N --> 500N', 'FitBoxToText','on')

% tiledlayout(1, 3)
% 
% % FX
% nexttile
% hold on
% box, grid on
% plot3(TEST_INPUT(:, 1), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
% legend('COMBINED FX')
% xlabel('SL')
% ylabel('FZ')
% zlabel('FX')
% 
% nexttile
% hold on
% box, grid on
% plot3(TEST_INPUT(:, 2), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
% legend('COMBINED FX')
% xlabel('SA')
% ylabel('FZ')
% zlabel('FX')
% 
% nexttile
% hold on
% box, grid on
% plot3(TEST_INPUT(:, 4), TEST_INPUT(:, 3), tireForces.COMBINED.FX, 'ro')
% legend('COMBINED FX')
% xlabel('IA')
% ylabel('FZ')
% zlabel('FX')

%% SL LIMIT 
close all

% TESTSL = linspace(-0.12, 0.12, 100)';
TESTSA = linspace(10,    10,   100)';
% TESTFZ = -linspace(700, 0, 100)';

TESTIA = linspace(0,     0,    100)';

TESTSL = linspace(-1, 1, 100)';

figure
hold on
grid on
xlabel('SL')
ylabel('FZ')
zlabel('FX')


for fz = -300:-100:-1000

    TESTFZ = repelem(fz, 100)';

    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);

    plot3(TESTSL, TESTFZ, tireForces.COMBINED.FX, 'ro')
    
    drawnow
end

%% SA LIMIT 
close all

% TESTSL = linspace(-0.12, 0.12, 100)';
TESTSA = linspace(-20,    20,   100)';
% TESTFZ = -linspace(700, 0, 100)';

TESTIA = linspace(0,     0,    100)';

TESTSL = linspace(.2, .2, 100)';

figure
hold on
grid on
xlabel('SA')
ylabel('FZ')
zlabel('FY')


for fz = -300:-100:-1000

    TESTFZ = repelem(fz, 100)';

    TEST_INPUT = [TESTSL, TESTSA, TESTFZ, TESTIA];
    
    tireForces = getTireForces(TEST_INPUT);

    plot3(TESTSA, TESTFZ, tireForces.COMBINED.FY, 'ro')
    
    drawnow
end



