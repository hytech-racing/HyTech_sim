%% HYTECH PACEJKA 92 (MF 5.2) MODIFIED TO TAVD 2ND ED.

% REFER TO... 
%   PACEJKA H.B. - TYRE AND VEHICLE DYNAMICS (TAVD) 2ND ED. CHAP. 4
%   FORMULA NOTES IN FOLDER
%   TTC FORUM - ESPECIALLY POSTS FROM BILLCOBB

% RE-EVALUATE IF CHANGING RUN NUMBER

% HOOSIER 16x7.5-10 LC0 - PURE SIDE SLIP - FY, MX, MZ - 8PSI
% ROUND 8, RUN 16

% HOOSIER 18x7.5-10 R25B - PURE LONGITUDINAL SLIP - FX - COMBINED - 8PSI 
% ROUND 6, RUN 36

clear global;
clear;
clc;
close all;

%% READ AND RUN EACH SECTION ONE BY ONE, CAREFULLY!!

%  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%  --------------------------------------------------
%% INITIALIZE PACEJKA 5.2 VARIABLES, PARAMETERS, AND SCALING FACTORS

% CONVENTION: 
% SAE FOR TTC TIRE AXIS SYSTEM 
% ADAPTED SAE FOR PACEJKA MODEL

% !!!!! SEE TTC TIRE AXIS SYSTEM.png FOR SA DIRECTION !!!!!
% !!!!! SEE PACEJKA AXIS SYSTEM PT1 & 2.png FOR SA DIRECTION !!!!!!

% ALPHA = SA, GAMMA = IA, KAPPA = LONGITUDINAL SLIP

% ALL ANGLES USED IN MODEL PARAM. DETERMINATION = RADIAN

% Variables
global FZO RO VCX

% PURE SIDE SLIP FY PARAMETERS
global PCY1 ...
       PDY1 PDY2 PDY3 ...
       PEY1 PEY2 PEY3 PEY4 PEY5...
       PKY1 PKY2 PKY3 PKY4 PKY5 PKY6 PKY7...
       PHY1 PHY2 ...
       PVY1 PVY2 PVY3 PVY4

% PURE LONG. SLIP FX PARAMETERS
global PCX1 ...
       PDX1 PDX2 PDX3 ...
       PEX1 PEX2 PEX3 PEX4 ...
       PKX1 PKX2 PKX3 ...
       PHX1 PHX2 ...
       PVX1 PVX2

% PURE SIDE SLIP MZ PARAMETERS
global QBZ1 QBZ2 QBZ3 QBZ5 QBZ6 QBZ9 QBZ10...
       QCZ1 ...
       QDZ1 QDZ2 QDZ3 QDZ4 QDZ6 QDZ7 QDZ8 QDZ9 QDZ10 QDZ11 ...
       QEZ1 QEZ2 QEZ3 QEZ4 QEZ5 ...
       QHZ1 QHZ2 QHZ3 QHZ4

% COMBINED FY DEPENDENT MX PARAMETERS 
global QSX1 QSX2 QSX3

% COMBINED FY DEPENDENT MX PARAMETERS FROM TEXT SECTION 4.3.5
global QX1 QX2 QX3 QX4 QX5 QX6

% COMBINED FX PARAMETERS
global RBX1 RBX2 RBX3 ...
       RCX1 ...
       REX1 REX2 ...
       RHX1

% COMBINED FY PARAMETERS
global RBY1 RBY2 RBY3 RBY4 ...
       RCY1 ...
       REY1 REY2 ...
       RHY1 RHY2 ...
       RVY1 RVY2 RVY3 RVY4 RVY5 RVY6

% COMBINED MZ PARAMETERS
global SSZ1 SSZ2 SSZ3 SSZ4
        

% Spin factor (ZETA0 -> ZETA8 = UNITY WHEN NEGLECTING TURN SLIP AND CAMBER REMAINS SMALL)
global ZETA0 ZETA1 ZETA2 ZETA3 ZETA4 ZETA5 ZETA6 ZETA7 ZETA8

% PREVENT SINGULARITY WHEN DENOMINATOR = 0
global EPSILON

%Scaling factors
% Pure slip
global LFZO LMUX LMUY LMUV LKXKAPPA LKYALPHA LCX LCY...
       LEX LEY LHX LHY LVX LVY LKYGAMMA LKZGAMMA ...
       LT LMR AMU LMUXAST LMUYAST LMUYPRIME LMUXPRIME

% Combined
global LXALPHA LYKAPPA LVYKAPPA LS

% Other
global LCZ LMX LMY
%% HOOSIER 16x7.5-10 LC0 - FY, MX, MZ - 8PSI
load B1965run16.mat

%% Overview Plot 
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
V = V*kmh2mps;
P = P*kPa2psi;

figure
tiledlayout(7,1)

ax1 = nexttile;
plot(ET, P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(ET, IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(ET, FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(ET, SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(ET, SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(ET, FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(ET, V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7], 'x')

sgtitle('Overview')
%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

% MF 5.2 Pure FY uses three inputs: ALPHA (Slip angle), FZ, and GAMMA
% (Inclination angle)
% Pure side slip

% Run 16 utilized a W2 warm up schedule, exclude first +/-12 SA
close all

idx     = ET > 130 & ET < 650;
EText   = ET(idx);
SAext   = SA(idx);
FZext   = FZ(idx);
FYext   = FY(idx);
MZext   = MZ(idx);
MXext   = MX(idx);
RLext   = RL(idx);
IAext   = IA(idx);
Pext    = P(idx);

tiledlayout(2,2) 
ax1 = nexttile; 
plot(EText,Pext) 
ylabel('Inflation Pressure [psi]') 
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(EText,IAext) 
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile; 
plot(EText,FZext) 
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(EText,SAext) 
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

sgtitle('EXTRACTED DATA')

linkaxes([ax1 ax2 ax3 ax4], 'x')

figure
plot3(SAext, FZext, FYext, 'ko')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('EXTRACTED DATA')

%% 'POLISH' DATA
close all

% Fit a spline using all points and find the zeros
m = 1:length(SAext);
sp = spline(m, SAext);
z = fnzeros(sp);
z = round(z(1, :));
z = [1, z, length(SAext)]; % hardcode in the first and last zeros

% Check the zeros
figure
hold on
plot(m, SAext)
plot(z, zeros(length(z)), 'ko')
yline(0)
xlabel('Data Point')
ylabel('SA [deg]')
title('SA Points for Segmentation')

% New data structure row counter
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

%% POLISH COMPARISON
close all;

tiledlayout(1, 2)
ax1 = nexttile;
hold on
grid on
box on
plot3(SAext, FZext, FYext, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('ORIGINAL')

ax2 = nexttile;
hold on
grid on
box on
plot3(ext.SA, ext.FZ, ext.FY, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('NEW')

linkprop([ax1 ax2], 'CameraPosition');
sgtitle('PURE SIDE SLIP FY COMPARISON')

%% IA INTUITION CHECK

% HOOSIER 16x7.5-10 LC0 DOES NOT LIKE IA
close all
idx_constFZ = abs(ext.FZ) > 600 & abs(ext.FZ) < 700;
figure
plot3(ext.SA(idx_constFZ), ext.IA(idx_constFZ), ext.FY(idx_constFZ), 'o')
xlabel('Slip Angle [deg]')
ylabel('IA [deg]')
zlabel('Lateral Force [N]')


%% PACEJKA MF 5.2 PARAM SET

% ADHERE TO ADAPTED SAE
FZO = 1100; % N, nominal load, highest tested normal load
RO = 0.2032; % m, unloaded tire radius
VCX = 10; % !! ONLY CONSIDERING POSITIVE LONGITUDINAL VEL., ARBITRARY MAGNITUDE!!

% PURE SLIP SCALING FACTORS
LFZO        = 1;
LMUX        = 1;
LMUY        = 1;
LKXKAPPA    = 1;
LKYALPHA    = 1;
LCX         = 1;
LCY         = 1;
LEX         = 1;
LEY         = 1;
LHX         = 1;
LHY         = 1;
LVX         = 1;
LVY         = 1;
LKYGAMMA    = 1;
LKZGAMMA    = 1;
LT          = 1;
LMR         = 1;

% OTHER - SCALING FACTOR
LMX         = 1;

% Allows simplification
LMUV        = 0;

% Simplification
AMU = 10; % Suggested value from text
LMUXAST = LMUX;
LMUYAST = LMUY;
LMUYPRIME = AMU .* LMUYAST ./ (1 + (AMU - 1) .* LMUYAST);
LMUXPRIME = AMU .* LMUXAST ./ (1 + (AMU - 1) .* LMUXAST);

% Spin Factor
ZETA0 = 1;
ZETA1 = 1;
ZETA2 = 1;
ZETA3 = 1;
ZETA4 = 1;
ZETA5 = 1;
ZETA6 = 1;
ZETA7 = 1;
ZETA8 = 1;

% Not expecting any denominator to become 0
EPSILON = 0;

%% PACEJKA MF 5.2 LATERAL FORCE (PURE SIDE SLIP)

% INPUTS TO MODEL = [SA, FZ, IA]

INPUT = [ext.SA, ext.FZ, ext.IA];

% EQUATIONS ACROSS DIFFERENT PLATFORMS DISAGREE, COEFFS. SHOULD NOT BE 
% USED TO DETERMINE CORNERING STIFFNESS DIRECTLY. 
% USE COEFFS. --> LIMIT SA --> FIND CORNERING STIFFNESS VIA POLYFIT
% FIT AT IA (GAMMA) = 0;

% LATERAL FORCE COEFFICIENTS INITIAL GUESS

% PCY1 = 2;
% PDY1 = 2;
% PDY2 = 0.4;
% PDY3 = 0.5;
% PEY1 = 1.2106;
% PEY2 = 0.22914;
% PEY3 = 0.031406;
% PEY4 = -0.88535;
% PEY5 = 2;
% PKY1 = -80;
% PKY2 = -1.948;
% PKY3 = -10;
% PKY4 = 0.22631;     
% PKY5 = 0.1;         
% PKY6 = 5.1824;
% PKY7 = -0.019021;
% PHY1 = 0.00038416;
% PHY2 = -0.00058327;
% PVY1 = -0.017621;
% PVY2 = -0.01;
% PVY3 = -3.4379;
% PVY4 = 1.653;


% PCY1 = 0.8;
% PDY1 = 2;
% PDY2 = -0.245;
% PDY3 = -1.23;
% PEY1 = -100.003;
% PEY2 = -100.37;
% PEY3 = -100.3;
% PEY4 = -100.787;
% PEY5 = 1;           % UNUSED
% PKY1 = -50;
% PKY2 = 2.13;
% PKY3 = -0.028;
% PKY4 = 2;     % UNUSED
% PKY5 = 0;         % UNUSED
% PKY6 = -0.92;
% PKY7 = -0.24;
% PHY1 = 0.003;
% PHY2 = -0.001;
% PVY1 = 0.045;
% PVY2 = -0.024;
% PVY3 = -0.532;
% PVY4 = 0.039;

% PCY1 = 0.016723;
% PDY1 = -205.2209;
% PDY2 = -6.1154;
% PDY3 = 9.7378;
% PEY1 = 1.2132;
% PEY2 = 0.27557;
% PEY3 = -0.031321;
% PEY4 = 0.78855;
% PEY5 = 13.2328;      % UNUSED    
% PKY1 = -710.9497;
% PKY2 = -0.61382;
% PKY3 = -13.0602;
% PKY4 = 0.056803;     
% PKY5 = -44.3034;         
% PKY6 = 5.1487;
% PKY7 = 0.25072;
% PHY1 = -0.00041405;
% PHY2 = 0.0004013;
% PVY1 = -0.014468;
% PVY2 = -0.013551;
% PVY3 = -3.3676;
% PVY4 = 1.6685;

% PCY1 = 1.193;
% PDY1 = -0.99;
% PDY2 = 0.145;
% PDY3 = -11.23;
% PEY1 = -1.003;
% PEY2 = -0.537;
% PEY3 = -0.083;
% PEY4 = -4.784;
% PEY5 = 13.2328;         
% PKY1 = -14.95;
% PKY2 = 2.130;
% PKY3 = -0.028;
% PKY4 = 2;     
% PKY5 = 0;         
% PKY6 = -0.92;
% PKY7 = -0.24;
% PHY1 = 0.003;
% PHY2 = -0.001;
% PVY1 = 0.045;
% PVY2 = -0.024;
% PVY3 = -0.532;
% PVY4 = 0.039;

PCY1 = 1.2314;
PDY1 = -2.6218;
PDY2 = 0.49409;
PDY3 = -0.11562;
PEY1 = -0.00020039;
PEY2 = -6.1623e-05;
PEY3 = -6462.7526;
PEY4 = -8275.9082;
PEY5 = -183385.9681;         
PKY1 = -7321.1048;
PKY2 = 0.42783;
PKY3 = -13.9291;
PKY4 = 0.0033467;     
PKY5 = 32.4742;         
PKY6 = -5.3924;
PKY7 = -1.1437;
PHY1 = 0.013038;
PHY2 = 0.011296;
PVY1 = 0.39277;
PVY2 = 0.11759;
PVY3 = 2.4196;
PVY4 = -2.2799;

% PCY1 = 1;
% PDY1 = 1;
% PDY2 = 1;
% PDY3 = 1;
% PEY1 = 1;
% PEY2 = 1;
% PEY3 = 1;
% PEY4 = 1;
% PEY5 = 1;         
% PKY1 = 1;
% PKY2 = 1;
% PKY3 = 1;
% PKY4 = 1;     
% PKY5 = 1;         
% PKY6 = 1;
% PKY7 = 1;
% PHY1 = 1;
% PHY2 = 1;
% PVY1 = 1;
% PVY2 = 1;
% PVY3 = 1;
% PVY4 = 1;

clear AA RESNORM

A_str = {'PCY1' 'PDY1' 'PDY2' 'PDY3' 'PEY1' 'PEY2' 'PEY3' ...
         'PEY4' 'PEY5' 'PKY1' 'PKY2' 'PKY3' 'PKY4' 'PKY5' ...
         'PKY6' 'PKY7' 'PHY1' 'PHY2' 'PVY1' 'PVY2' 'PVY3' ...
         'PVY4'};

% Keeps track of coefficients
A_old = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 ...
         PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 ...
         PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 ...
         PVY4];

options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig1 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 FY Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [A, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_PSS_FY', A_old, INPUT, ext.FY, [], [], options);

    AA(:, run) = A;

    for n = 1:22
        subplot(2, 11, n)
        bar(AA(n, :), 'group')
        title(['A(' num2str(n) ')' ' =' A_str{n}], 'FontSize', 8)
    end

    for n = 1:22
        disp(['A_old(' num2str(n) ') = ' num2str(A_old(n)) ';    ' 'A(' num2str(n) ') = ' num2str(A(n)) ';'])
        eval(['A_old(' num2str(n) ') = ' num2str(A(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end

%% CHECK PACEJKA PURE SIDE SLIP FY FIT
close all

% FY vs. SA vs. FZ
figure
tiledlayout(1, 3)
sgtitle('FITTING COMPARISON AT 8 PSI')
fy_fit = Pacejka52_PSS_FY(A, INPUT);
nexttile
hold on
grid on
box on
plot3(INPUT(:, 1),INPUT(:, 2), ext.FY, 'k.')
plot3(INPUT(:, 1),INPUT(:, 2), fy_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load (N)')
zlabel('Lateral Force [N]')
title('FY vs. SA vs. FZ, FITTING COMPARISON AT 8 PSI')
legend('Data Pts','FITTED DATA')


% NFY vs. SA vs. FZ

fy_fit = Pacejka52_PSS_FY(A, INPUT);
nexttile
hold on
grid on
box on
plot3(INPUT(:, 1), INPUT(:, 2), ext.FY./abs(INPUT(:, 2)), 'k.')
plot3(INPUT(:, 1), INPUT(:, 2), fy_fit./abs(INPUT(:, 2)), 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load (N)')
zlabel('NFY - Dimensionless')
title('NFY vs. SA vs. FZ')
legend('Data Pts','FITTED DATA')

% FY vs. SA vs. IA, comparison with original figure
INPUT_constFZ = [ext.SA, ones(size(ext.SA)) .* -650, ext.IA];
fy_fit_IACheck = Pacejka52_PSS_FY(A, INPUT_constFZ);
nexttile
hold on
grid on
box on

idx_constFZ = abs(ext.FZ) > 600 & abs(ext.FZ) < 700;
plot3((ext.SA(idx_constFZ)), ext.IA(idx_constFZ), (ext.FY(idx_constFZ)./650), 'o')
plot3((INPUT_constFZ(:, 1)), INPUT_constFZ(:, 3), (fy_fit_IACheck./650), 'o')

xlabel('Slip Angle [deg]')
ylabel('IA [deg]')
zlabel('NFY - Dimensionless')

legend('ORIGINAL', 'FITTED DATA')


%% ACCEPT LATERAL COEFFICIENTS

PCY1 = A(1);
PDY1 = A(2);
PDY2 = A(3);
PDY3 = A(4);
PEY1 = A(5);
PEY2 = A(6);
PEY3 = A(7);
PEY4 = A(8);
PEY5 = A(9);
PKY1 = A(10);
PKY2 = A(11);
PKY3 = A(12);
PKY4 = A(13); 
PKY5 = A(14); 
PKY6 = A(15);
PKY7 = A(16);
PHY1 = A(17);
PHY2 = A(18);
PVY1 = A(19);
PVY2 = A(20);
PVY3 = A(21);
PVY4 = A(22);

%% CHECK PURE SIDE SLIP MZ DATA
close all

tiledlayout(1, 2)
ax1 = nexttile;
hold on
box, grid on
plot3(SAext, FZext, MZext, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
title('ORIGINAL')

ax2 = nexttile;
hold on
box, grid on
plot3(ext.SA, ext.FZ, ext.MZ, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
title('NEW')

linkprop([ax1 ax2], 'CameraPosition');
sgtitle('PURE SIDE SLIP MZ COMPARISON')


%% PACEJKA MF 5.2 ALIGNING TORQUE (PURE SIDE SLIP)

% INPUTS TO MODEL = [SA, FZ, IA]

INPUT = [ext.SA, ext.FZ, ext.IA];

% ALIGNING TORQUE COEFFICIENTS INITIAL GUESS, FROM TEXT
QBZ1    = 9;
QBZ2    = -1.1;
QBZ3    = -0.8;
QBZ5    = -0.227;
QBZ6    = 0;
QBZ9    = 18.47;
QBZ10   = 0;
QCZ1    = 1.18;
QDZ1    = 0.1;
QDZ2    = -0.001;
QDZ3    = 0.007;
QDZ4    = 13.05;
QDZ6    = -0.008;
QDZ7    = 0;
QDZ8    = -0.296;
QDZ9    = -0.009;
QDZ10   = 0;
QDZ11   = 0;
QEZ1    = -1.609;
QEZ2    = -0.359;
QEZ3    = 0;
QEZ4    = 0.174;
QEZ5    = -0.896;  
QHZ1    = 0.007;
QHZ2    = -0.002;
QHZ3    = 0.147;
QHZ4    = 0.004;

clear BB RESNORM

B_str = {'QBZ1' 'QBZ2' 'QBZ3'  'QBZ5'  'QBZ6' 'QBZ9' 'QBZ10' ...
         'QCZ1' 'QDZ1' 'QDZ2'  'QDZ3'  'QDZ4' 'QDZ6' 'QDZ7' ...
         'QDZ8' 'QDZ9' 'QDZ10' 'QDZ11' 'QEZ1' 'QEZ2' 'QEZ3' ...
         'QEZ4' 'QEZ5' 'QHZ1'  'QHZ2'  'QHZ3' 'QHZ4'};

% Keeps track of coefficients
B_old = [QBZ1 QBZ2 QBZ3  QBZ5  QBZ6 QBZ9 QBZ10 ...
         QCZ1 QDZ1 QDZ2  QDZ3  QDZ4 QDZ6 QDZ7 ...
         QDZ8 QDZ9 QDZ10 QDZ11 QEZ1 QEZ2 QEZ3 ...
         QEZ4 QEZ5 QHZ1  QHZ2  QHZ3 QHZ4];

options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig2 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 MZ Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [B, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_PSS_MZ', B_old, INPUT, ext.MZ, [], [], options);

    BB(:, run) = B;

    for n = 1:27
        subplot(3, 9, n)
        bar(BB(n, :), 'group')
        title(['B(' num2str(n) ')' ' =' B_str{n}], 'FontSize', 8)
    end

    for n = 1:27
        disp(['B_old(' num2str(n) ') = ' num2str(B_old(n)) ';    ' 'B(' num2str(n) ') = ' num2str(B(n)) ';'])
        eval(['B_old(' num2str(n) ') = ' num2str(B(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end

%% CHECK PACEJKA PURE SIDE SLIP MZ FIT
close all

% MZ vs. SA vs. FZ
figure
mz_fit = Pacejka52_PSS_MZ(B, INPUT);
hold on
box on
grid on
plot3(INPUT(:, 1),INPUT(:, 2), ext.MZ, 'k.')
plot3(INPUT(:, 1),INPUT(:, 2), mz_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load (N)')
zlabel('Aligning Torque [N-m]')
legend('Data Pts','FITTED DATA')
title('MZ vs. SA vs. FZ, FITTING COMPARISON AT 8 PSI')

%% ACCEPT ALIGNING TORQUE COEFFS.

QBZ1    = B(1);
QBZ2    = B(2);
QBZ3    = B(3);
QBZ5    = B(4);
QBZ6    = B(5);
QBZ9    = B(6);
QBZ10   = B(7);
QCZ1    = B(8);
QDZ1    = B(9);
QDZ2    = B(10);
QDZ3    = B(11);
QDZ4    = B(12);
QDZ6    = B(13);
QDZ7    = B(14);
QDZ8    = B(15);
QDZ9    = B(16);
QDZ10   = B(17);
QDZ11   = B(18);
QEZ1    = B(19);
QEZ2    = B(20);
QEZ3    = B(21);
QEZ4    = B(22);
QEZ5    = B(23);  
QHZ1    = B(24);
QHZ2    = B(25);
QHZ3    = B(26);
QHZ4    = B(27);




%% SEPARATING SECTION, SWAP TIRE FOR FX, SAVE ALL COEFFICIENTS BEFORE CONTINUING

%{


      ╱|、
    (˚ˎ 。7  
     |、˜〵          
     じしˍ,)ノ



    |\__/,|   (`\
  _.|o o  |_   ) )
-(((---(((--------



............... ／＞　 フ
................| _　 _l
.............／` ミ＿xノ 
.........../　　　　 |
........../　 ヽ　　 ﾉ
.........│　　|　| |
....／￣|　　 |　| |
....| (￣ヽ＿_ヽ_)__)
.....＼二つ



%}




%% HOOSIER 18x7.5-10 R25B
clear;
clc;
load B1654run36.mat



%% Overview Plot
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
V = V*kmh2mps;
P = P*kPa2psi;

figure
tiledlayout(4,2)

ax1 = nexttile;
plot(ET, P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(ET, IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(ET, FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(ET, SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(ET, SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(ET, V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(ET, FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax8 = nexttile;
plot(ET, FY)
ylabel('FY [N]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'x')

sgtitle('Overview')

%% IA INTUITION CHECK

% HOOSIER 16x7.5-10 LC0 DOES NOT LIKE IA, COMBINED DATA TIRE SHOULDN'T
% EITHER

close all
idx_constFZ = abs(FZ) > 600 & abs(FZ) < 800;
figure
hold on
grid on
plot3(SA(idx_constFZ), IA(idx_constFZ), FY(idx_constFZ), 'o')
xlabel('Slip Angle [deg]')
ylabel('IA [deg]')
zlabel('Lateral Force [N]')

%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

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

figure
tiledlayout(3,2) 
ax1 = nexttile; 
plot(EText,Pext) 
ylabel('Inflation Pressure [psi]') 
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(EText,IAext) 
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile; 
plot(EText,FZext) 
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(EText,SAext) 
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(EText,SLext) 
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(EText, FXext)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

sgtitle('EXTRACTED DATA')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6], 'x')

figure;
hold on
grid on
plot3(SLext, FZext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Normal Load [N]')
zlabel('Longitudinal Force [N]')
title('EXTRACTED DATA')

%% FIND SEGMENTATION POINTS
close all

% SEGMENT BASED ON FZ
m = length(FZext);
% sp = spline(m, SLext);
% z = ischange(SLext, 'linear');
% z = [1 m(z) length(SLext)];

z = [];

for ind = 1 : m - 1

    currFZ = FZext(ind);
    nextFZ = FZext(ind + 1);

    FZTol = 100;

    if (nextFZ < (currFZ - FZTol) || nextFZ > (currFZ + FZTol))

        z = [z, ind + 1];

    end
end

% Hardcode in starting and ending locations
z = [1, z, m];

% Check the zeros
tiledlayout(2, 1)
nexttile
hold on
plot((1: m), SLext)
xline(z)
xlabel('Data Point')
ylabel('SL [deg]')
title('SL Points for Segmentation')

nexttile
hold on
plot((1: m), FZext)
xline(z)
xlabel('Data Point')
ylabel('FZ [N]')
title('FZ Points for Segmentation')


%% 'POLISH' DATA


ptTol = 0;

% New data structure row counter
row = 0;

clear fmdata
clear ext

% Splits up data per SL segment
for n = 1 : length(z) - 1

    sa = SAext(z(n) : z(n + 1) - ptTol);
    fz = FZext(z(n) : z(n + 1) - ptTol);
    fy = FYext(z(n) : z(n + 1) - ptTol);
    mz = MZext(z(n) : z(n + 1) - ptTol);
    mx = MXext(z(n) : z(n + 1) - ptTol);
    rl = RLext(z(n) : z(n + 1) - ptTol);
    ia = IAext(z(n) : z(n + 1) - ptTol);
    sl = SLext(z(n) : z(n + 1) - ptTol);
    sr = SRext(z(n) : z(n + 1) - ptTol);
    fx = FXext(z(n) : z(n + 1) - ptTol);

    % fx = smoothdata(fx, 'sgolay');

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
    sp_fy = csaps(sa, fy);

    for slipRatio = (floor(min(sl) * 100) / 100) + 0.025 : 0.005 : (ceil(max(sl) * 100) / 100) - 0.025

        row = row + 1;
        fmdata(row, 1) = slipRatio;
        fmdata(row, 2) = round(mean(ia));
        fmdata(row, 3) = mean(fz);
        fmdata(row, 4) = fnval(sp_fx, slipRatio);
        fmdata(row, 5) = mean(sa);
        fmdata(row, 6) = fnval(sp_fy, mean(sa));

    end

    
end

[filteredFX, TF] = filloutliers(fmdata(:, 4), 'makima', 'movmedian', 5);
fmdata(:, 4) = filteredFX;

% Sorts rows based on IA -> SL -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);

ext.SL = fmdata(:,1);
ext.IA = fmdata(:,2);
ext.FZ = fmdata(:,3);
ext.FX = fmdata(:,4);
ext.SA = fmdata(:,5);
ext.FY = fmdata(:,6);


%% POLISH COMPARISON
close all

tiledlayout(1, 2)
ax1 = nexttile;
hold on
box, grid on
plot3(SLext, FZext, FXext, 'o')
xlabel('Longitudinal Slip')
ylabel('Normal Load [N]')
zlabel('Longitudinal Force [N]')
title('ORIGINAL')

ax2 = nexttile;
hold on
box, grid on
plot3(ext.SL, ext.FZ, ext.FX, 'o')
xlabel('Longitudinal Slip')
ylabel('Normal Load [N]')
zlabel('Logitudinal Force [N]')
title('NEW')

sgtitle('FX COMPARISON')




%% RE-RUN GLOBAL VARIABLE INITIALIZATION AND SETTING SECTIONS

% !!!  DO BOTH OF THEM  !!!

% INITIALIZE PACEJKA 5.2 VARIABLES, PARAMETERS, AND SCALING FACTORS
% PACEJKA MF 5.2 PARAM SET


%% PACEJKA MF 5.2 LONGITUDINAL FORCE (PURE LONGITUDINAL SLIP)

close all

% INPUTS TO MODEL = [SL, FZ, IA]

INPUT = [ext.SL, ext.FZ, ext.IA];

% LONGITUDINAL FORCE COEFFICIENTS INITIAL GUESS FROM TEXT
% PCX1 = -0.1;
% PDX1 = 1;
% PDX2 = -0.037;
% PDX3 = 1;
% PEX1 = 0.344;
% PEX2 = 0.095;
% PEX3 = -0.020;
% PEX4 = 0;
% PKX1 = 1;
% PKX2 = -0.163;
% PKX3 = 0.245;
% PHX1 = -0.002;
% PHX2 = 0.002;
% PVX1 = 0;
% PVX2 = 0;

% COEFFS. AFTER SECOND FIT
PCX1 = 1.2058;
PDX1 = 3.2199;
PDX2 = -1.4226;
PDX3 = 19.15;
PEX1 = 0.19292;
PEX2 = -3.1467;
PEX3 = -3.6617;
PEX4 = 0.12872;
PKX1 = 51.7788;
PKX2 = 0.0032173;
PKX3 = -0.67928;
PHX1 = 0.0013353;
PHX2 = -0.00033102;
PVX1 = -0.093762;
PVX2 = 0.06282;

clear DD RESNORM

D_str = {'PCX1' 'PDX1' 'PDX2' 'PDX3' 'PEX1' 'PEX2' 'PEX3' 'PEX4' ...
         'PKX1' 'PKX2' 'PKX3' 'PHX1' 'PHX2' 'PVX1' 'PVX2'};

% Keeps track of coefficients
D_old = [PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 ...
         PKX1 PKX2 PKX3 PHX1 PHX2 PVX1 PVX2];

options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig1 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 FX Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [D, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_PLS_FX', D_old, INPUT, ext.FX, [], [], options);

    DD(:, run) = D;

    for n = 1:15
        subplot(3, 5, n)
        bar(DD(n, :), 'group')
        title(['D(' num2str(n) ')' ' =' D_str{n}], 'FontSize', 8)
    end

    for n = 1:15
        disp(['D_old(' num2str(n) ') = ' num2str(D_old(n)) ';    ' 'D(' num2str(n) ') = ' num2str(D(n)) ';'])
        eval(['D_old(' num2str(n) ') = ' num2str(D(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end

%% CHECK PACEJKA FX FIT
close all

% FX vs. SL vs. FZ
figure
fx_fit = Pacejka52_PLS_FX(D, INPUT);
grid on
hold on
plot3(INPUT(:, 1),INPUT(:, 2), ext.FX, 'k.')
plot3(INPUT(:, 1),INPUT(:, 2), fx_fit, 'ro')
xlabel('Longitudinal Slip')
ylabel('Vertical Load (N)')
zlabel('Longitudinal Force (N)')
legend('Data Pts','FITTED DATA')
title('FX vs. SL vs. FZ, FITTING COMPARISON AT 8 PSI')

%% FX EXTRAPOLATION TREND CHECK

close all;

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

    fx_fit = Pacejka52_PLS_FX(D, [TESTSL, TESTFZ, repelem(0, 100)']);

    plot3(TESTSL, TESTFZ, fx_fit, 'ro')
    
    drawnow
end
%% ACCEPT LONGITUDINAL FORCE COEFFS.

PCX1 = D(1);
PDX1 = D(2);
PDX2 = D(3);
PDX3 = D(4);
PEX1 = D(5);
PEX2 = D(6);
PEX3 = D(7);
PEX4 = D(8);
PKX1 = D(9);
PKX2 = D(10);
PKX3 = D(11);
PHX1 = D(12);
PHX2 = D(13);
PVX1 = D(14);
PVX2 = D(15);
 

%% TIRES FITTED FOR PURE SLIP, CHEERS /ᐠ - ˕ -マ

%{

⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⣦⡀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⣾⣿⣿⣿⣷⠀⠀⠀   SAVE ALL COEFFS. BEFORE CONTINUING
⠀⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⣾⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀⠀⠀⢀⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣧⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀
⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣠⣤⣤⣼⣿⣿⣿⣿⣿⣿⣿⣿⣷⠀⠀⠀⠀⠀
⠀⠀⠀⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀
⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀
⠀⠀⠀⠘⣿⣿⣿⣿⠟⠁⠀⠀⠀⠹⣿⣿⣿⣿⣿⠟⠁⠀⠀⠹⣿⣿⡿⠀⠀⠀⠀⠀
⠀⠀⠀⠀⣿⣿⣿⡇⠀⠀⠀⢼⣿⠀⢿⣿⣿⣿⣿⠀⣾⣷⠀⠀⢿⣿⣷⠀⠀⠀⠀⠀
⠀⠀ ⢠⣿⣿⣿⣷⡀⠀⠀⠈⠋⢀⣿⣿⣿⣿⣿⡀⠙⠋⠀⢀⣾⣿⣿⠀⠀⠀⠀⠀
⢀⣀⣀⣀⣿⣿⣿⣿⣿⣶⣶⣶⣶⣿⣿⣿⣿⣾⣿⣷⣦⣤⣴⣿⣿⣿⣿⣤⠤⢤⣤⡄
⠈⠉⠉⢉⣙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⣀⣀⣀⡀⠀
⠐⠚⠋⠉⢀⣬⡿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣥⣀⡀⠈⠀⠈⠛
⠀⠀⠴⠚⠉⠀⠀⠀⠉⠛⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠛⠋⠁⠀⠀⠀⠉⠛⠢⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀

%}


%% COMBINED SLIP FITTING

clear; 
clc;

%% RE-RUN GLOBAL VARIABLE INITIALIZATION ONLY

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% JUST THE INITIALIZATION, DO NOT RUN PARAM. SET

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


%% GET PURE SLIP COEFFICIENTS

PSS_FYCOEFFS = load('PSS LATERAL FORCE COEFFS.mat').A;
PLS_FXCOEFFS = load('PLS LONGITUDINAL FORCE COEFFS.mat').D;
PSS_MZCOEFFS = load('PSS ALIGNING TORQUE COEFFS.mat').B;
% PSS_MXCOEFFS = load('PSS OVERTURNING MOMENT COEFFS.mat').C;

% SET COEFFS. FOR PURE SIDE SLIP FY
PCY1 = PSS_FYCOEFFS(1);
PDY1 = PSS_FYCOEFFS(2);
PDY2 = PSS_FYCOEFFS(3);
PDY3 = PSS_FYCOEFFS(4);
PEY1 = PSS_FYCOEFFS(5);
PEY2 = PSS_FYCOEFFS(6);
PEY3 = PSS_FYCOEFFS(7);
PEY4 = PSS_FYCOEFFS(8);
PEY5 = PSS_FYCOEFFS(9);
PKY1 = PSS_FYCOEFFS(10);
PKY2 = PSS_FYCOEFFS(11);
PKY3 = PSS_FYCOEFFS(12);
PKY4 = PSS_FYCOEFFS(13); 
PKY5 = PSS_FYCOEFFS(14); 
PKY6 = PSS_FYCOEFFS(15);
PKY7 = PSS_FYCOEFFS(16);
PHY1 = PSS_FYCOEFFS(17);
PHY2 = PSS_FYCOEFFS(18);
PVY1 = PSS_FYCOEFFS(19);
PVY2 = PSS_FYCOEFFS(20);
PVY3 = PSS_FYCOEFFS(21);
PVY4 = PSS_FYCOEFFS(22);

% SET COEFFS. FOR PURE LONG. SLIP FX
PCX1 = PLS_FXCOEFFS(1);
PDX1 = PLS_FXCOEFFS(2);
PDX2 = PLS_FXCOEFFS(3);
PDX3 = PLS_FXCOEFFS(4);
PEX1 = PLS_FXCOEFFS(5);
PEX2 = PLS_FXCOEFFS(6);
PEX3 = PLS_FXCOEFFS(7);
PEX4 = PLS_FXCOEFFS(8);
PKX1 = PLS_FXCOEFFS(9);
PKX2 = PLS_FXCOEFFS(10);
PKX3 = PLS_FXCOEFFS(11);
PHX1 = PLS_FXCOEFFS(12);
PHX2 = PLS_FXCOEFFS(13);
PVX1 = PLS_FXCOEFFS(14);
PVX2 = PLS_FXCOEFFS(15);

% SET COEFFS. FOR PURE SIDE SLIP MZ
QBZ1    = PSS_MZCOEFFS(1);
QBZ2    = PSS_MZCOEFFS(2);
QBZ3    = PSS_MZCOEFFS(3);
QBZ5    = PSS_MZCOEFFS(4);
QBZ6    = PSS_MZCOEFFS(5);
QBZ9    = PSS_MZCOEFFS(6);
QBZ10   = PSS_MZCOEFFS(7);
QCZ1    = PSS_MZCOEFFS(8);
QDZ1    = PSS_MZCOEFFS(9);
QDZ2    = PSS_MZCOEFFS(10);
QDZ3    = PSS_MZCOEFFS(11);
QDZ4    = PSS_MZCOEFFS(12);
QDZ6    = PSS_MZCOEFFS(13);
QDZ7    = PSS_MZCOEFFS(14);
QDZ8    = PSS_MZCOEFFS(15);
QDZ9    = PSS_MZCOEFFS(16);
QDZ10   = PSS_MZCOEFFS(17);
QDZ11   = PSS_MZCOEFFS(18);
QEZ1    = PSS_MZCOEFFS(19);
QEZ2    = PSS_MZCOEFFS(20);
QEZ3    = PSS_MZCOEFFS(21);
QEZ4    = PSS_MZCOEFFS(22);
QEZ5    = PSS_MZCOEFFS(23);  
QHZ1    = PSS_MZCOEFFS(24);
QHZ2    = PSS_MZCOEFFS(25);
QHZ3    = PSS_MZCOEFFS(26);
QHZ4    = PSS_MZCOEFFS(27);


%% HOOSIER 18x7.5-10 R25B
load B1654run36.mat



%% Overview Plot
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
V = V*kmh2mps;
P = P*kPa2psi;

figure
tiledlayout(4,2)

ax1 = nexttile;
plot(ET, P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(ET, IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(ET, FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(ET, SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(ET, SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(ET, V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(ET, FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax8 = nexttile;
plot(ET, FY)
ylabel('FY [N]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'x')

sgtitle('Overview')

%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

idx     = ET > 25 & ET < 582;
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
Vext    = V(idx);
Pext    = P(idx);

close all

figure
tiledlayout(4,2)

ax1 = nexttile;
plot(EText, Pext)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(EText, IAext)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(EText, FZext)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(EText, SLext)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(EText, SAext)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(EText, Vext)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(EText, FXext)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax8 = nexttile;
plot(EText, FYext)
ylabel('FY [N]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'x')

sgtitle('EXTRACTED Overview')

figure
tiledlayout(1, 3)
nexttile
hold on
grid on
plot3(SLext, FZext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Normal Load [N]')
zlabel('Longitudinal Force [N]')
title('FX vs. SL vs. FZ')

nexttile
hold on 
grid on
plot3(SLext, IAext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Inclination Angle [deg]')
zlabel('Longitudinal Force [N]')
title('FX vs. SL vs. IA')

nexttile
hold on
grid on
plot3(SLext, SAext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Slip Angle [deg]')
zlabel('Longitudinal Force [N]')
title('FX vs. SL vs. SA')

sgtitle('EXTRACTED Data')



%% FIND SEGMENTATION POINTS
close all
% SEGMENT BASED ON FZ
m = length(FZext);
% sp = spline(m, SLext);
% z = ischange(SLext, 'linear');
% z = [1 m(z) length(SLext)];

z = [];

for ind = 1 : m - 1

    currFZ = FZext(ind);
    nextFZ = FZext(ind + 1);

    FZTol = 100;

    if (nextFZ < (currFZ - FZTol) || nextFZ > (currFZ + FZTol))

        z = [z, ind + 1];

    end
end

% Hardcode in starting and ending locations
z = [1, z, m];

% Check the zeros
tiledlayout(2, 1)
nexttile
hold on
plot((1: m), SLext)
xline(z)
xlabel('Data Point')
ylabel('SL [deg]')
title('SL Points for Segmentation')

nexttile
hold on
plot((1: m), FZext)
xline(z)
xlabel('Data Point')
ylabel('FZ [N]')
title('FZ Points for Segmentation')

%% 'POLISH' DATA

ptTol = 0;

% New data structure row counter
row = 0;

clear fmdata
clear ext

% Splits up data per SL segment
for n = 1 : length(z) - 1

    sa = SAext(z(n) : z(n + 1) - ptTol);
    fz = FZext(z(n) : z(n + 1) - ptTol);
    fy = FYext(z(n) : z(n + 1) - ptTol);
    mz = MZext(z(n) : z(n + 1) - ptTol);
    mx = MXext(z(n) : z(n + 1) - ptTol);
    rl = RLext(z(n) : z(n + 1) - ptTol);
    ia = IAext(z(n) : z(n + 1) - ptTol);
    sl = SLext(z(n) : z(n + 1) - ptTol);
    sr = SRext(z(n) : z(n + 1) - ptTol);
    fx = FXext(z(n) : z(n + 1) - ptTol);

    % fx = smoothdata(fx, 'sgolay');

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
    sp_fy = csaps(sa, fy);

    for slipRatio = (floor(min(sl) * 100) / 100) + 0.005 : 0.005 : (ceil(max(sl) * 100) / 100) - 0.001

        row = row + 1;
        fmdata(row, 1) = slipRatio;
        fmdata(row, 2) = round(mean(ia));
        fmdata(row, 3) = mean(fz);
        fmdata(row, 4) = fnval(sp_fx, slipRatio);
        fmdata(row, 5) = mean(sa);
        fmdata(row, 6) = fnval(sp_fy, mean(sa));

    end

    
end

[filteredFX, TF] = filloutliers(fmdata(:, 4), 'makima', 'movmedian', 5);
fmdata(:, 4) = filteredFX;

% Sorts rows based on IA -> SL -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);

ext.SL = fmdata(:,1);
ext.IA = fmdata(:,2);
ext.FZ = fmdata(:,3);
ext.FX = fmdata(:,4);
ext.SA = fmdata(:,5);
ext.FY = fmdata(:,6);

%% POLISH COMPARISON
close all;

figure
tiledlayout(3, 3)
nexttile
hold on
grid on
plot3(SLext, FZext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Normal Load [N]')
zlabel('Longitudinal Force [N]')
title('ORIGINAL FX vs. SL vs. FZ')

nexttile
hold on
grid on
plot3(ext.SL, ext.FZ, ext.FX, 'ro')
xlabel('Longitudinal Slip (SL)')
ylabel('Normal Load [N]')
zlabel('Longitudinal Force [N]')
title('NEW FX vs. SL vs. FZ')

nexttile
hold on
grid on
plot3(SLext, FZext, FXext, 'k.')
plot3(ext.SL, ext.FZ, ext.FX, 'ro')
xlabel('Longitudinal Slip (SL)')
ylabel('Normal Load [N]')
zlabel('Longitudinal Force [N]')
title('COMPARISON FX vs. SL vs. FZ')

nexttile
hold on 
grid on
plot3(SLext, IAext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Inclination Angle [deg]')
zlabel('Longitudinal Force [N]')
title('ORIGINAL FX vs. SL vs. IA')

nexttile
hold on 
grid on
plot3(ext.SL, ext.IA, ext.FX, 'ro')
xlabel('Longitudinal Slip (SL)')
ylabel('Inclination Angle [deg]')
zlabel('Longitudinal Force [N]')
title('NEW FX vs. SL vs. IA')

nexttile
hold on 
grid on
plot3(SLext, IAext, FXext, 'k.')
plot3(ext.SL, ext.IA, ext.FX, 'ro')
xlabel('Longitudinal Slip (SL)')
ylabel('Inclination Angle [deg]')
zlabel('Longitudinal Force [N]')
title('COMPARISON FX vs. SL vs. IA')

nexttile
hold on
grid on
plot3(SLext, SAext, FXext, 'ko')
xlabel('Longitudinal Slip (SL)')
ylabel('Slip Angle [deg]')
zlabel('Longitudinal Force [N]')
title('ORIGINAL FX vs. SL vs. SA')

nexttile
hold on
grid on
plot3(ext.SL, ext.SA, ext.FX, 'ro')
xlabel('Longitudinal Slip (SL)')
ylabel('Slip Angle [deg]')
zlabel('Longitudinal Force [N]')
title('NEW FX vs. SL vs. SA')

nexttile
hold on
grid on
plot3(SLext, SAext, FXext, 'k.')
plot3(ext.SL, ext.SA, ext.FX, 'ro')
plot3(ext.SL, -ext.SA, ext.FX, 'b*')
xlabel('Longitudinal Slip (SL)')
ylabel('Slip Angle [deg]')
zlabel('Longitudinal Force [N]')
title('COMPARISON FX vs. SL vs. SA')

%%

ext.SLtest = [ext.SL; ext.SL];
ext.SAtest = [ext.SA; -ext.SA];
ext.FZtest = [ext.FZ; ext.FZ];
ext.IAtest = [ext.IA; ext.IA];
ext.FXtest = [ext.FX; ext.FX];

figure
hold on
grid on
plot3(ext.SL, ext.SA, ext.FX, 'ro')
plot3(ext.SLtest, ext.SAtest, ext.FXtest, 'b*')
xlabel('Longitudinal Slip (SL)')
ylabel('Slip Angle [deg]')
zlabel('Longitudinal Force [N]')

% figure
% hold on
% grid on
% plot3(ext.SL, ext.FZ, ext.FX, 'ro')
% plot3(ext.SLtest, ext.FZtest, ext.FXtest, 'b*')
% xlabel('Longitudinal Slip (SL)')
% ylabel('Normal Load [N]')
% zlabel('Longitudinal Force [N]')
% title('COMPARISON FX vs. SL vs. FZ')

%% PACEJKA MF 5.2 PARAM SET, ENSURE THESE ARE EXACTLY THE SAME AS FIRST PARAM SET

% ADHERE TO ADAPTED SAE
FZO = 1100; % N, nominal load, highest tested normal load
RO = 0.2032; % m, unloaded tire radius

% VCX ONLY USED FOR SIGN
VCX = 10; % !! CURRENTLY ONLY CONSIDERING POSITIVE LONGITUDINAL VEL., ARBITRARY MAGNITUDE !!

% PURE SLIP SCALING FACTORS
LFZO        = 1;
LMUX        = 1;
LMUY        = 1;
LKXKAPPA    = 1;
LKYALPHA    = 1;
LCX         = 1;
LCY         = 1;
LEX         = 1;
LEY         = 1;
LHX         = 1;
LHY         = 1;
LVX         = 1;
LVY         = 1;
LKYGAMMA    = 1;
LKZGAMMA    = 1;
LT          = 1;
LMR         = 1;

% COMBINED - SCALING FACTOR
LXALPHA     = 1;
LYKAPPA     = 1;
LVYKAPPA    = 1;
LS          = 1;

% OTHER - SCALING FACTOR
LMX         = 1;

% Allows simplification
LMUV        = 0;

% Simplification
AMU = 10; % Suggested value from text
LMUXAST = LMUX;
LMUYAST = LMUY;
LMUYPRIME = AMU .* LMUYAST ./ (1 + (AMU - 1) .* LMUYAST);
LMUXPRIME = AMU .* LMUXAST ./ (1 + (AMU - 1) .* LMUXAST);

% Spin Factor
ZETA0 = 1;
ZETA1 = 1;
ZETA2 = 1;
ZETA3 = 1;
ZETA4 = 1;
ZETA5 = 1;
ZETA6 = 1;
ZETA7 = 1;
ZETA8 = 1;

% Not expecting any denominator to become 0
EPSILON = 0;

%% PACEJKA MF 5.2 LONGITUDINAL FORCE (COMBINED SLIP)

close all

% INPUTS TO MODEL = [SL, SA, FZ, IA]

INPUT = [ext.SLtest, ext.SAtest, ext.FZtest, ext.IAtest];

SANearZero = abs(ext.SA) >= 0 & abs(ext.SA) <= 1;
ext.SA(SANearZero) = 0;

% LONGITUDINAL FORCE COEFFICIENTS INITIAL GUESS
RBX1 = 0.8;
RBX2 = 1.075;
RBX3 = 1.5;
RCX1 = 1.092;
REX1 = 0.5;
REX2 = -2;
RHX1 = 0;

% LONG. FORCE COEFFS. FROM FIRST FIT
% RBX1 = 1.92899749;
% RBX2 = 1.02472319340594;
% RBX3 = .1;
% RCX1 = -1.78057797756512;
% REX1 = 1.00000068456070;
% REX2 = -1.90991244424283e-08;
% RHX1 = -0.00265696417448022;

clear EE RESNORM

E_str = {'RBX1' 'RBX2' 'RBX3' 'RCX1' 'REX1' 'REX2' 'RHX1'};

% Keeps track of coefficients
E_old = [RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1];

options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig1 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 COMBINED FX Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [E, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_COMBINED_FX', E_old, INPUT, ext.FXtest, [], [], options);

    EE(:, run) = E;

    for n = 1:7
        subplot(3, 3, n)
        bar(EE(n, :), 'group')
        title(['E(' num2str(n) ')' ' =' E_str{n}], 'FontSize', 8)
    end

    for n = 1:7
        disp(['E_old(' num2str(n) ') = ' num2str(E_old(n)) ';    ' 'E(' num2str(n) ') = ' num2str(E(n)) ';'])
        eval(['E_old(' num2str(n) ') = ' num2str(E(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end

%% CHECK PACEJKA FX FIT
% close all

fx_fit = Pacejka52_COMBINED_FX(E, INPUT);

fx_fit1 = Pacejka52_COMBINED_FX(E, [INPUT(:, 1) -1.*ones(size(INPUT(:, 1))) INPUT(:, 3) INPUT(:, 4)]);
fx_fit2 = Pacejka52_COMBINED_FX(E, [INPUT(:, 1) -5.*ones(size(INPUT(:, 1))) INPUT(:, 3) INPUT(:, 4)]);
fx_fit3 = Pacejka52_COMBINED_FX(E, [INPUT(:, 1) -10.*ones(size(INPUT(:, 1))) INPUT(:, 3) INPUT(:, 4)]);
fx_fit4 = Pacejka52_COMBINED_FX(E, [INPUT(:, 1)  1.*ones(size(INPUT(:, 1))) INPUT(:, 3) INPUT(:, 4)]);
fx_fit5 = Pacejka52_COMBINED_FX(E, [INPUT(:, 1)  5.*ones(size(INPUT(:, 1))) INPUT(:, 3) INPUT(:, 4)]);
fx_fit6 = Pacejka52_COMBINED_FX(E, [INPUT(:, 1)  10.*ones(size(INPUT(:, 1))) INPUT(:, 3) INPUT(:, 4)]);


PLS_INPUT = [ext.SLtest, ext.FZtest, ext.IAtest];
fx_PLS_fit = Pacejka52_PLS_FX(PLS_FXCOEFFS, PLS_INPUT);

% FX vs. SL vs. FZ
figure
tiledlayout(1, 3)
ax1 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 1), INPUT(:, 3), ext.FXtest, 'k.')
plot3(INPUT(:, 1), INPUT(:, 3), fx_fit, 'ro')
plot3(PLS_INPUT(:, 1), PLS_INPUT(:, 2), fx_PLS_fit, 'b*')
xlabel('Longitudinal Slip')
ylabel('Vertical Load [N]')
zlabel('Longitudinal Force [N]')
legend('Data Pts','FITTED DATA', 'PURE FIT')
title('FX vs. SL vs. FZ')

% FX vs. SL vs. SA
ax2 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 1), INPUT(:, 2), ext.FXtest, 'k.')
plot3(INPUT(:, 1), INPUT(:, 2), fx_fit, 'ro')
plot3(PLS_INPUT(:, 1), zeros(size(PLS_INPUT(:, 1))), fx_PLS_fit, 'b*')

plot3(PLS_INPUT(:, 1), -1.*ones(size(INPUT(:, 1))), fx_fit1, 'b*')
plot3(PLS_INPUT(:, 1), -5.*ones(size(INPUT(:, 1))), fx_fit2, 'b*')
plot3(PLS_INPUT(:, 1), -10.*ones(size(INPUT(:, 1))), fx_fit3, 'b*')
plot3(PLS_INPUT(:, 1),  1.*ones(size(INPUT(:, 1))), fx_fit4, 'b*')
plot3(PLS_INPUT(:, 1),  5.*ones(size(INPUT(:, 1))), fx_fit5, 'b*')
plot3(PLS_INPUT(:, 1),  10.*ones(size(INPUT(:, 1))), fx_fit6, 'b*')

xlabel('Longitudinal Slip')
ylabel('Slip Angle [deg]')
zlabel('Longitudinal Force [N]')
legend('Data Pts','FITTED DATA', 'PURE FIT + DISCRETE SA FIT')
title('FX vs. SL vs. SA')

% FX vs. SL vs. IA
ax3 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 1), INPUT(:, 4), ext.FXtest, 'k.')
plot3(INPUT(:, 1), INPUT(:, 4), fx_fit, 'ro')
plot3(PLS_INPUT(:, 1), PLS_INPUT(:, 3), fx_PLS_fit, 'b*')
xlabel('Longitudinal Slip')
ylabel('IA [deg]')
zlabel('Longitudinal Force [N]')
legend('Data Pts','FITTED DATA', 'PURE FIT')
title('FX vs. SL vs. IA')

%% FX EXTRAPOLATION TREND CHECK

% close all;

TESTIA = linspace(0, 0, 100)';
TESTSA = linspace(12, 0, 100)';
TESTSL = linspace(-1, 1, 100)';

figure
hold on
grid on
xlabel('SL')
ylabel('FZ')
zlabel('FX')


for fz = -300:-100:-1000

    TESTFZ = repelem(fz, 100)';

    fx_fit = Pacejka52_COMBINED_FX(E, [TESTSL, TESTSA, TESTFZ, repelem(0, 100)']);

    plot3(TESTSL, TESTFZ, fx_fit, 'ro')
    
    drawnow
end

%% ACCEPT COMBINED FX COEFFS.

RBX1 = E(1);
RBX2 = E(2);
RBX3 = E(3);
RCX1 = E(4);
REX1 = E(5);
REX2 = E(6);
RHX1 = E(7);

%% COMBINED SLIP FY 

% SINCE LC0 DOES NOT HAVE COMBINED FY DATA

% PURE SIDE SLIP R25B IS COMPARED TO PURE SIDE SLIP LC0 TO FIND A SCALING
% FACTOR

% THIS SCALING FACTOR IS THEN APPLIED TO R25B COMBINED SLIP

%% LOAD BOTH DATA

LC0 = load('B1965run16.mat');
R25B = load('B1654run25.mat'); % FOR CORNERING TEST

%% LC0 Overview Plot 
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
LC0.V = LC0.V*kmh2mps;
LC0.P = LC0.P*kPa2psi;

figure
tiledlayout(7,1)

ax1 = nexttile;
plot(LC0.ET, LC0.P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(LC0.ET, LC0.IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(LC0.ET, LC0.FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(LC0.ET, LC0.SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(LC0.ET, LC0.SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(LC0.ET, LC0.FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(LC0.ET, LC0.V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7], 'x')

sgtitle('LC0 OVERVIEW')
%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

close all

idx         = LC0.ET > 130 & LC0.ET < 650;
LC0.EText   = LC0.ET(idx);
LC0.SAext   = LC0.SA(idx);
LC0.FZext   = LC0.FZ(idx);
LC0.FYext   = LC0.FY(idx);
LC0.MZext   = LC0.MZ(idx);
LC0.MXext   = LC0.MX(idx);
LC0.RLext   = LC0.RL(idx);
LC0.IAext   = LC0.IA(idx);
LC0.Pext    = LC0.P(idx);

tiledlayout(2,2) 
ax1 = nexttile; 
plot(LC0.EText,LC0.Pext) 
ylabel('Inflation Pressure [psi]') 
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(LC0.EText,LC0.IAext) 
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile; 
plot(LC0.EText,LC0.FZext) 
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(LC0.EText,LC0.SAext) 
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

sgtitle('LC0 EXTRACTED DATA')

linkaxes([ax1 ax2 ax3 ax4], 'x')

figure
plot3(LC0.SAext, LC0.FZext, LC0.FYext, 'ko')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('LC0 EXTRACTED DATA')

%% 'POLISH' DATA
close all
clear fmdata dmdata

% Fit a spline using all points and find the zeros
m = 1:length(LC0.SAext);
sp = spline(m, LC0.SAext);
z = fnzeros(sp);
z = round(z(1, :));
z = [1, z, length(LC0.SAext)]; % hardcode in the first and last zeros

% Check the zeros
figure
hold on
plot(m, LC0.SAext)
plot(z, zeros(length(z)), 'ko')
yline(0)
xlabel('Data Point')
ylabel('SA [deg]')
title('SA Points for Segmentation')

% New data structure row counter
fmrow = 0;
dmrow = 0;

% Splits up data per +/-12 deg SA segment
for n = 1:2:length(z) - 1
    sa = LC0.SAext(z(n) : z(n + 2));
    fz = LC0.FZext(z(n) : z(n + 2));
    fy = LC0.FYext(z(n) : z(n + 2));
    mz = LC0.MZext(z(n) : z(n + 2));
    mx = LC0.MXext(z(n) : z(n + 2));
    rl = LC0.RLext(z(n) : z(n + 2));
    ia = LC0.IAext(z(n) : z(n + 2));

    fy = mean(fz) .* fy ./ fz;
    mz = mean(fz) .* mz ./ fz;
    mx = mean(fz) .* mx ./ fz;
    fz = mean(fz) .* ones(length(fz), 1);

    sp_fy = csaps(sa, fy, 0.1);
    sp_mz = csaps(sa, mz, 0.1);
    sp_mx = csaps(sa, mx, 0.1);
    sp_rl = csaps(sa, rl, 0.1);

    for slipAng = -6 : 0.25 : 0

        fmrow = fmrow + 1;
        fmdata(fmrow, 1) = slipAng;
        fmdata(fmrow, 2) = round(mean(ia));
        fmdata(fmrow, 3) = mean(fz);
        fmdata(fmrow, 4) = fnval(sp_fy, slipAng);
        fmdata(fmrow, 5) = fnval(sp_mz, slipAng);
        fmdata(fmrow, 6) = fnval(sp_mx, slipAng);

    end

    for slipAng = 6 : -0.25 : 0

        dmrow = dmrow + 1;
        dmdata(dmrow, 1) = slipAng;
        dmdata(dmrow, 2) = round(mean(ia));
        dmdata(dmrow, 3) = mean(fz);
        dmdata(dmrow, 4) = fnval(sp_fy, slipAng);
        dmdata(dmrow, 5) = fnval(sp_mz, slipAng);
        dmdata(dmrow, 6) = fnval(sp_mx, slipAng);

    end
end

% Sorts rows based on IA -> SA -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);
dmdata = sortrows(dmdata, [2, 1, 3]);

LC0ext.SA       = fmdata(:,1);
LC0ext.SAPOS    = dmdata(:,1);
LC0ext.IA       = fmdata(:,2);
LC0ext.FZ       = fmdata(:,3);
LC0ext.FZSAPOS  = dmdata(:,3);
LC0ext.FY       = fmdata(:,4);
LC0ext.FYSAPOS  = dmdata(:,4);
LC0ext.MZ       = fmdata(:,5);
LC0ext.MZSAPOS  = dmdata(:,5);
LC0ext.MX       = fmdata(:,6);

%% POLISH COMPARISON
close all;

figure
hold on
box on
grid on
plot3(LC0ext.SA, LC0ext.FZ, LC0ext.FY, 'o')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.FYSAPOS, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
legend('NEGATIVE SA', 'POSITIVE SA')
title('LC0 PURE SIDE SLIP FY ACTUAL DATA')

%% R25B Overview Plot 
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
R25B.V = R25B.V*kmh2mps;
R25B.P = R25B.P*kPa2psi;

figure
tiledlayout(7,1)

ax1 = nexttile;
plot(R25B.ET, R25B.P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(R25B.ET, R25B.IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(R25B.ET, R25B.FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(R25B.ET, R25B.SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(R25B.ET, R25B.SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(R25B.ET, R25B.FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(R25B.ET, R25B.V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7], 'x')

sgtitle('R25B OVERVIEW')
%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

close all

idx          = R25B.ET > 123 & R25B.ET < 665;
R25B.EText   = R25B.ET(idx);
R25B.SAext   = R25B.SA(idx);
R25B.FZext   = R25B.FZ(idx);
R25B.FYext   = R25B.FY(idx);
R25B.MZext   = R25B.MZ(idx);
R25B.MXext   = R25B.MX(idx);
R25B.RLext   = R25B.RL(idx);
R25B.IAext   = R25B.IA(idx);
R25B.Pext    = R25B.P(idx);

tiledlayout(2,2) 
ax1 = nexttile; 
plot(R25B.EText,R25B.Pext) 
ylabel('Inflation Pressure [psi]') 
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(R25B.EText,R25B.IAext) 
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile; 
plot(R25B.EText,R25B.FZext) 
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(R25B.EText,R25B.SAext) 
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

sgtitle('R25B EXTRACTED DATA')

linkaxes([ax1 ax2 ax3 ax4], 'x')

figure
plot3(R25B.SAext, R25B.FZext, R25B.FYext, 'ko')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('R25B EXTRACTED DATA')

%% 'POLISH' DATA
close all
clear fmdata

% Fit a spline using all points and find the zeros
m = 1:length(R25B.SAext);
sp = spline(m, R25B.SAext);
z = fnzeros(sp);
z = round(z(1, :));
z = [1, z, length(R25B.SAext)]; % hardcode in the first and last zeros

% Check the zeros
figure
hold on
plot(m, R25B.SAext)
plot(z, zeros(length(z)), 'ko')
yline(0)
xlabel('Data Point')
ylabel('SA [deg]')
title('SA Points for Segmentation')

% New data structure row counter
row = 0;

% Splits up data per +/-12 deg SA segment
for n = 1:2:length(z) - 1
    sa = R25B.SAext(z(n) : z(n + 2));
    fz = R25B.FZext(z(n) : z(n + 2));
    fy = R25B.FYext(z(n) : z(n + 2));
    mz = R25B.MZext(z(n) : z(n + 2));
    mx = R25B.MXext(z(n) : z(n + 2));
    rl = R25B.RLext(z(n) : z(n + 2));
    ia = R25B.IAext(z(n) : z(n + 2));

    fy = mean(fz) .* fy ./ fz;
    mz = mean(fz) .* mz ./ fz;
    mx = mean(fz) .* mx ./ fz;
    fz = mean(fz) .* ones(length(fz), 1);

    sp_fy = csaps(sa, fy, 0.1);
    sp_mz = csaps(sa, mz, 0.1);
    sp_mx = csaps(sa, mx, 0.1);
    sp_rl = csaps(sa, rl, 0.1);

    for slipAng = -6 : 0.25 : 0

        row = row + 1;
        fmdata(row, 1) = slipAng;
        fmdata(row, 2) = round(mean(ia));
        fmdata(row, 3) = mean(fz);
        fmdata(row, 4) = fnval(sp_fy, slipAng);
        fmdata(row, 5) = fnval(sp_mz, slipAng);
        fmdata(row, 6) = fnval(sp_mx, slipAng);

    end
end

% Sorts rows based on IA -> SA -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);

R25Bext.SA = fmdata(:,1);
R25Bext.IA = fmdata(:,2);
R25Bext.FZ = fmdata(:,3);
R25Bext.FY = fmdata(:,4);
R25Bext.MZ = fmdata(:,5);
R25Bext.MX = fmdata(:,6);

%% POLISH COMPARISON

% Only for negative SA

close all;

figure
plot3(R25Bext.SA, R25Bext.FZ, R25Bext.FY, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
sgtitle('R25B PURE SIDE SLIP FY ACTUAL DATA ONLY NEG SA')

%% LC0 VS. R25B Comparison
close all
figure
hold on
box on
grid on
plot3(LC0ext.SA,  LC0ext.FZ,  LC0ext.FY, 'bo')
plot3(R25Bext.SA, R25Bext.FZ, R25Bext.FY, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('LC0 vs. R25B')
legend('LC0', 'R25B')

%% GET SCALING FACTOR
R25BtoLC0FY = LC0ext.FY ./ R25Bext.FY;
R25BtoLC0MZ = LC0ext.MZ ./ R25Bext.MZ;

R25Bext.scaledFY = R25Bext.FY .* R25BtoLC0FY;
R25Bext.scaledMZ = R25Bext.MZ .* R25BtoLC0MZ;

close all
figure
tiledlayout(1, 2)
sgtitle('R25B --> LC0 SCALING TEST')
nexttile
hold on
box on
grid on
plot3(LC0ext.SA,  LC0ext.FZ,  LC0ext.FY, 'bo')
plot3(R25Bext.SA, R25Bext.FZ, R25Bext.scaledFY, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
legend('LC0', 'SCALED R25B')
title('FY')

nexttile
hold on
box on
grid on
plot3(LC0ext.SA,  LC0ext.FZ,  LC0ext.MZ, 'bo')
plot3(R25Bext.SA, R25Bext.FZ, R25Bext.scaledMZ, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N]')
legend('LC0', 'SCALED R25B')
title('MZ')

%% ARTIFICIALLY CONSTRUCT POSITIVE SA

LC0ext.MIRRORFY = -LC0ext.FY;
LC0ext.MIRRORMZ = -LC0ext.MZ;
LC0ext.MIRRORSA = -LC0ext.SA;


% temp = LC0ext.MIRRORSA;
% newVec = zeros(size(temp));
% slipAngleTo = 6;
% 
% 
% for slipAngleFrom = 0 : 0.25 : 6
%     mask = temp == slipAngleFrom;
%     newVec(mask) = slipAngleTo;
%     slipAngleTo = slipAngleTo - 0.25;
% end
% 
% LC0ext.ARTIFICIALSA = newVec;

LC0MirroredFYtoFY = LC0ext.FYSAPOS ./ LC0ext.MIRRORFY;
LC0ext.ARTIFICIALFY = LC0ext.MIRRORFY .* LC0MirroredFYtoFY;

LC0MirroredMZtoMZ = LC0ext.MZSAPOS ./ LC0ext.MIRRORMZ;
LC0ext.ARTIFICIALMZ = LC0ext.MIRRORMZ .* LC0MirroredMZtoMZ;

close all;

tiledlayout(2, 2)
sgtitle('LC0 PURE SIDE SLIP COMPARISON')
nexttile
hold on
box on
grid on
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.FYSAPOS, 'o')
plot3(LC0ext.MIRRORSA, LC0ext.FZSAPOS, LC0ext.MIRRORFY, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
legend('ACTUAL FY', 'MIRRORED FY')

nexttile
hold on
box on
grid on
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.FYSAPOS, 'o')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.ARTIFICIALFY, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
legend('ACTUAL FY', 'ARTIFICIAL SCALED FY')

nexttile
hold on
box on
grid on
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.MZSAPOS, 'o')
plot3(LC0ext.MIRRORSA, LC0ext.FZSAPOS, LC0ext.MIRRORMZ, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
legend('ACTUAL MZ', 'MIRRORED MZ')

nexttile
hold on
box on
grid on
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.MZSAPOS, 'o')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.ARTIFICIALMZ, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
legend('ACTUAL MZ', 'ARTIFICIAL SCALED MZ')



%% SCALING FACTOR CHECK
close all
LC0ext.CONSTRUCTEDSA = [R25Bext.SA; LC0ext.SAPOS];
LC0ext.CONSTRUCTEDFZ = [R25Bext.FZ; LC0ext.FZSAPOS];
LC0ext.CONSTRUCTEDFY = [R25Bext.scaledFY; LC0ext.ARTIFICIALFY];
LC0ext.CONSTRUCTEDMZ = [R25Bext.scaledMZ; LC0ext.ARTIFICIALMZ];

tiledlayout(2, 2)
nexttile
hold on
box on
grid on
plot3(LC0ext.SA, LC0ext.FZ, LC0ext.FY, 'bo')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.FYSAPOS, 'bo')
plot3(R25Bext.SA, R25Bext.FZ, R25Bext.scaledFY, 'r*')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.ARTIFICIALFY, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
legend('ACTUAL LC0 FY NEG SA', 'ACTUAL LC0 FY POS SA', 'R25B --> LC0 NEG SA', 'R25B --> LC0 NEG SA --> MIRROR --> LC0 POS SA')

nexttile
hold on
box on
grid on
plot3(LC0ext.SA, LC0ext.FZ, LC0ext.FY, 'bo')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.FYSAPOS, 'bo')
plot3(LC0ext.CONSTRUCTEDSA, LC0ext.CONSTRUCTEDFZ, LC0ext.CONSTRUCTEDFY, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
legend('ACTUAL LC0 FY NEG SA', 'ACTUAL LC0 FY POS SA', 'CONSTRUCTED FY')

nexttile
hold on
box on
grid on
plot3(LC0ext.SA, LC0ext.FZ, LC0ext.MZ, 'bo')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.MZSAPOS, 'bo')
plot3(R25Bext.SA, R25Bext.FZ, R25Bext.scaledMZ, 'r*')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.ARTIFICIALMZ, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
legend('ACTUAL LC0 MZ NEG SA', 'ACTUAL LC0 MZ POS SA', 'R25B --> LC0 NEG SA', 'R25B --> LC0 NEG SA --> MIRROR --> LC0 POS SA')

nexttile
hold on
box on
grid on
plot3(LC0ext.SA, LC0ext.FZ, LC0ext.MZ, 'bo')
plot3(LC0ext.SAPOS, LC0ext.FZSAPOS, LC0ext.MZSAPOS, 'bo')
plot3(LC0ext.CONSTRUCTEDSA, LC0ext.CONSTRUCTEDFZ, LC0ext.CONSTRUCTEDMZ, 'r*')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
legend('ACTUAL LC0 MZ NEG SA', 'ACTUAL LC0 MZ POS SA', 'CONSTRUCTED MZ')


%% NOW TO TAKE THESE SCALING FACTORS AND CONSTRUCT +/- 6 deg SA FOR COMBINED FY, MZ

% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
% /////////////////////////////////////////////////////////////////////////////////
%% HOOSIER 18x7.5-10 R25B
load B1654run36.mat



%% Overview Plot
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
V = V*kmh2mps;
P = P*kPa2psi;

figure
tiledlayout(4,2)

ax1 = nexttile;
plot(ET, P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(ET, IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(ET, FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(ET, V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(ET, SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(ET, FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(ET, SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax8 = nexttile;
plot(ET, FY)
ylabel('FY [N]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'x')

sgtitle('Overview')

%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

idx     = ET > 25 & ET < 582;
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
Vext    = V(idx);
Pext    = P(idx);

close all

figure
tiledlayout(4,2)

ax1 = nexttile;
plot(EText, Pext)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(EText, IAext)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(EText, FZext)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(EText, SLext)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(EText, SAext)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(EText, Vext)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(EText, FXext)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax8 = nexttile;
plot(EText, FYext)
ylabel('FY [N]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8], 'x')

sgtitle('EXTRACTED Overview')

figure
tiledlayout(1, 3)
nexttile
hold on
grid on
plot3(SAext, FZext, FYext, 'ko')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('FX vs. SA vs. FZ')

nexttile
hold on 
grid on
plot3(SAext, IAext, FYext, 'ko')
xlabel('Slip Angle [deg]')
ylabel('Inclination Angle [deg]')
zlabel('Lateral Force [N]')
title('FX vs. SA vs. IA')

nexttile
hold on
grid on
plot3(SAext, SLext, FYext, 'ko')
ylabel('Longitudinal Slip (SL)')
xlabel('Slip Angle [deg]')
zlabel('Lateral Force [N]')
title('FX vs. SA vs. SL')

sgtitle('EXTRACTED Data')



%% FIND SEGMENTATION POINTS
close all
% SEGMENT BASED ON FZ
m = length(FZext);
% sp = spline(m, SLext);
% z = ischange(SLext, 'linear');
% z = [1 m(z) length(SLext)];

z = [];

for ind = 1 : m - 1

    currFZ = FZext(ind);
    nextFZ = FZext(ind + 1);

    FZTol = 100;

    if (nextFZ < (currFZ - FZTol) || nextFZ > (currFZ + FZTol))

        z = [z, ind + 1];

    end
end

% Hardcode in starting and ending locations
z = [1, z, m];

% Check the zeros
tiledlayout(2, 1)
nexttile
hold on
plot((1: m), SAext)
xline(z)
xlabel('Data Point')
ylabel('SA [deg]')
title('SA Points for Segmentation')

nexttile
hold on
plot((1: m), FZext)
xline(z)
xlabel('Data Point')
ylabel('FZ [N]')
title('FZ Points for Segmentation')

%% 'POLISH' DATA

ptTol = 30;

% New data structure row counter
row = 0;

clear fmdata
clear ext

newSA = [];
newFZ = [];
newFY = [];
newMZ = [];
newIA = [];
newSL = [];
newFX = [];

% Splits up data per SA segment
for n = 1 : length(z) - 1

    sa = SAext(z(n) + ptTol : z(n + 1) - ptTol);
    fz = FZext(z(n) + ptTol : z(n + 1) - ptTol);
    fy = FYext(z(n) + ptTol : z(n + 1) - ptTol);
    mz = MZext(z(n) + ptTol : z(n + 1) - ptTol);
    mx = MXext(z(n) + ptTol : z(n + 1) - ptTol);
    rl = RLext(z(n) + ptTol : z(n + 1) - ptTol);
    ia = IAext(z(n) + ptTol : z(n + 1) - ptTol);
    sl = SLext(z(n) + ptTol : z(n + 1) - ptTol);
    sr = SRext(z(n) + ptTol : z(n + 1) - ptTol);
    fx = FXext(z(n) + ptTol : z(n + 1) - ptTol);

    % fx = smoothdata(fx, 'sgolay');

    % tiledlayout(3, 1)
    % nexttile
    % plot(fz)
    % nexttile
    % plot(sl)
    % nexttile
    % plot(fx)

%     fx = abs(mean(fz)) .* fx ./ abs(fz);
%     fy = abs(mean(fz)) .* fy ./ abs(fz);
%     mz = abs(mean(fz)) .* mz ./ abs(fz);
%     fz = mean(fz) .* ones(length(fz), 1);
% 
%     newSA = [newSA; sa];
%     newFZ = [newFZ; fz];
%     newFY = [newFY; fy];
%     newMZ = [newMZ; mz];
%     newIA = [newIA; ia];
%     newSL = [newSL; sl];
%     newFX = [newFX; fx];
% 
% 
% 
% 
%     % for slipRatio = (floor(min(sl) * 100) / 100) + 0.005 : 0.005 : (ceil(max(sl) * 100) / 100) - 0.005
%     % 
%     %     row = row + 1;
%     %     fmdata(row, 1) = slipRatio;
%     %     fmdata(row, 2) = round(mean(ia));
%     %     fmdata(row, 3) = mean(fz);
%     %     fmdata(row, 4) = fnval(sp_fx, slipRatio);
%     %     fmdata(row, 5) = mean(sa);
%     %     fmdata(row, 6) = fnval(sp_fy, mean(sa));
%     % 
%     % end
% 
% 
% end
% 
% % [filteredFX, TF] = filloutliers(fmdata(:, 4), 'makima', 'movmedian', 5);
% % fmdata(:, 4) = filteredFX;
% % 
% % % Sorts rows based on IA -> SL -> FZ
% % fmdata = sortrows(fmdata, [2, 1, 3]);
% 
% ext.SA = newSA;
% ext.IA = newIA;
% ext.FZ = newFZ;
% ext.FY = newFY;
% ext.MZ = newMZ;
% ext.SL = newSL;
% ext.FX = newFX;


    fx = abs(mean(fz)) .* fx ./ abs(fz);
    fy = abs(mean(fz)) .* fy ./ abs(fz);
    mz = abs(mean(fz)) .* mz ./ abs(fz);
    fz = mean(fz) .* ones(length(fz), 1);
    
    sp_fx = csaps(sl, fx);
    sp_fy = csaps(sl, fy);
    sp_mz = csaps(sl, mz);

    for slipRatio = (floor(min(sl) * 100) / 100) + 0.01 : 0.01 : (ceil(max(sl) * 100) / 100) - 0.01

        row = row + 1;
        fmdata(row, 1) = slipRatio;
        fmdata(row, 2) = round(mean(ia));
        fmdata(row, 3) = mean(fz);
        fmdata(row, 4) = fnval(sp_fx, slipRatio);
        fmdata(row, 5) = mean(sa);
        fmdata(row, 6) = fnval(sp_fy, slipRatio);
        fmdata(row, 7) = fnval(sp_mz, slipRatio);

    end

    
end

[filteredFX, TF] = filloutliers(fmdata(:, 4), 'makima', 'movmedian', 5);
fmdata(:, 4) = filteredFX;
[filteredFY, TF] = filloutliers(fmdata(:, 6), 'makima', 'movmedian', 5);
fmdata(:, 6) = filteredFY;
[filteredMZ, TF] = filloutliers(fmdata(:, 7), 'makima', 'movmedian', 5);
fmdata(:, 7) = filteredMZ;



% Sorts rows based on IA -> SL -> FZ
fmdata = sortrows(fmdata, [2, 1, 3]);

ext.SL = fmdata(:,1);
ext.IA = fmdata(:,2);
ext.FZ = fmdata(:,3);
ext.FX = fmdata(:,4);
ext.SA = fmdata(:,5);
ext.FY = fmdata(:,6);
ext.MZ = fmdata(:,7);

%% POLISH COMPARISON
close all;

figure
tiledlayout(2, 3)


nexttile
hold on
grid on
plot3(SAext, FZext, FYext, 'k.')
plot3(ext.SA, ext.FZ, ext.FY, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('COMPARISON FY vs. SA vs. FZ')


nexttile
hold on 
grid on
plot3(SAext, IAext, FYext, 'k.')
plot3(ext.SA, ext.IA, ext.FY, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Inclination Angle [deg]')
zlabel('Lateral Force [N]')
title('COMPARISON FY vs. SA vs. IA')


nexttile
hold on
grid on
plot3(SAext, SLext, FYext, 'k.')
plot3(ext.SA, ext.SL, ext.FY, 'ro')
ylabel('Longitudinal Slip (SL)')
xlabel('Slip Angle [deg]')
zlabel('Lateral Force [N]')
title('COMPARISON FY vs. SA vs. SL') 

nexttile
hold on
grid on
plot3(SAext, FZext, MZext, 'k.')
plot3(ext.SA, ext.FZ, ext.MZ, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
title('COMPARISON MZ vs. SA vs. FZ')


nexttile
hold on 
grid on
plot3(SAext, IAext, MZext, 'k.')
plot3(ext.SA, ext.IA, ext.MZ, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Inclination Angle [deg]')
zlabel('Aligning Torque [N-m]')
title('COMPARISON MZ vs. SA vs. IA')


nexttile
hold on
grid on
plot3(SAext, SLext, MZext, 'k.')
plot3(ext.SA, ext.SL, ext.MZ, 'ro')
ylabel('Longitudinal Slip (SL)')
xlabel('Slip Angle [deg]')
zlabel('Aligning Torque [N-m]')
title('COMPARISON MZ vs. SA vs. SL') 

%% APPLY SCALING FACTORS
close all

% NOT ACTUALLY AT -6, -3, 0
% WHEN SETTING SLIP TO RANGE FROM [-6:3:0], neg0Scale BASICALLY BECOMES
% AVERAGE 0

% THE CURRENT SCHEME SEEMS TO PROVIDE COMPARABLE RESULT TO SCALING 
% AT ACTUAL -6 AND -3

neg6ScaleFY = [];
neg3ScaleFY = [];
neg0ScaleFY = [];

neg6ScaleMZ = [];
neg3ScaleMZ = [];
neg0ScaleMZ = [];

for x = 1:15:length(R25BtoLC0FY) - 15
    neg6ScaleFY = [neg6ScaleFY; R25BtoLC0FY(x : x+4)];
    neg3ScaleFY = [neg3ScaleFY; R25BtoLC0FY(x+5 : x+9)];
    neg0ScaleFY = [neg0ScaleFY; R25BtoLC0FY(x+10 : x+14)];

    neg6ScaleMZ = [neg6ScaleMZ; R25BtoLC0MZ(x : x+4)];
    neg3ScaleMZ = [neg3ScaleMZ; R25BtoLC0MZ(x+5 : x+9)];
    neg0ScaleMZ = [neg0ScaleMZ; R25BtoLC0MZ(x+10 : x+14)];
end

neg6ScaleFY = mean(rmoutliers(neg6ScaleFY));
neg3ScaleFY = mean(rmoutliers(neg3ScaleFY));
neg0ScaleFY = mean(rmoutliers(neg0ScaleFY));

neg6ScaleMZ = mean(rmoutliers(neg6ScaleMZ));
neg3ScaleMZ = mean(rmoutliers(neg3ScaleMZ));
neg0ScaleMZ = mean(rmoutliers(neg0ScaleMZ));

neg6Mask = abs(ext.SA) > 5;
neg3Mask = abs(ext.SA) > 2 & abs(ext.SA) < 5;
neg0Mask = abs(ext.SA) > -1 & abs(ext.SA) < 2;

ext.SCALEDFY = ext.FY;
ext.SCALEDFY(neg6Mask) = ext.SCALEDFY(neg6Mask) .* neg6ScaleFY;
ext.SCALEDFY(neg3Mask) = ext.SCALEDFY(neg3Mask) .* neg3ScaleFY;
ext.SCALEDFY(neg0Mask) = ext.SCALEDFY(neg0Mask) .* neg0ScaleFY;

ext.SCALEDMZ = ext.MZ;
ext.SCALEDMZ(neg6Mask) = ext.SCALEDMZ(neg6Mask) .* neg6ScaleFY;
ext.SCALEDMZ(neg3Mask) = ext.SCALEDMZ(neg3Mask) .* neg3ScaleFY;
ext.SCALEDMZ(neg0Mask) = ext.SCALEDMZ(neg0Mask) .* neg0ScaleFY;

figure
tiledlayout(1,2)
nexttile
hold on
plot3(ext.SA, ext.SL, ext.FY, 'bo')
plot3(ext.SA, ext.SL, ext.SCALEDFY, 'r*')
legend('Original', 'Scaled')
xlabel('Slip Angle [deg]')
ylabel('Longitudinal Slip (SL)')
zlabel('Lateral Force [N]')

nexttile
hold on
plot3(ext.SA, ext.SL, ext.MZ, 'bo')
plot3(ext.SA, ext.SL, ext.SCALEDMZ, 'r*')
legend('Original', 'Scaled')
xlabel('Slip Angle [deg]')
ylabel('Longitudinal Slip (SL)')
zlabel('Aligning Torque [N-m]')

sgtitle('Scaling Comparison R25B --> LC0')

%% ARTIFICIAL FORCE CONSTRUCTION

aboveZeroSA = abs(ext.SA) > 1;

ext.MIRRORSA = -ext.SA(aboveZeroSA);

% SLIP RATIO NOT FLIPPED
ext.MIRRORSL = ext.SL(aboveZeroSA);

% FZ NOT FLIPPED
ext.MIRRORFZ = ext.FZ(aboveZeroSA);

% IA NOT FLIPPED
ext.MIRRORIA = ext.IA(aboveZeroSA);


pos6ScaleFY = [];
pos3ScaleFY = [];
pos0ScaleFY = [];

pos6ScaleMZ = [];
pos3ScaleMZ = [];
pos0ScaleMZ = [];

for x = 1:15:length(LC0MirroredFYtoFY) - 15
    pos6ScaleFY = [pos6ScaleFY; LC0MirroredFYtoFY(x : x+4)];
    pos3ScaleFY = [pos3ScaleFY; LC0MirroredFYtoFY(x+5 : x+9)];
    pos0ScaleFY = [pos0ScaleFY; LC0MirroredFYtoFY(x+10 : x+14)];

    pos6ScaleMZ = [pos6ScaleMZ; LC0MirroredMZtoMZ(x : x+4)];
    pos3ScaleMZ = [pos3ScaleMZ; LC0MirroredMZtoMZ(x+5 : x+9)];
    pos0ScaleMZ = [pos0ScaleMZ; LC0MirroredMZtoMZ(x+10 : x+14)];
end

pos6ScaleFY = mean(rmoutliers(pos6ScaleFY));
pos3ScaleFY = mean(rmoutliers(pos3ScaleFY));
pos0ScaleFY = mean(rmoutliers(pos0ScaleFY));

pos6ScaleMZ = mean(rmoutliers(pos6ScaleMZ));
pos3ScaleMZ = mean(rmoutliers(pos3ScaleMZ));
pos0ScaleMZ = mean(rmoutliers(pos0ScaleMZ));

pos6Mask = abs(ext.MIRRORSA) > 5;
pos3Mask = abs(ext.MIRRORSA) > 2 & abs(ext.MIRRORSA) < 5;
pos0Mask = abs(ext.MIRRORSA) > -1 & abs(ext.MIRRORSA) < 2;

ext.FYSAPOS = -ext.SCALEDFY(aboveZeroSA);
ext.FYSAPOS(pos6Mask) = ext.FYSAPOS(pos6Mask) .* pos6ScaleFY;
ext.FYSAPOS(pos3Mask) = ext.FYSAPOS(pos3Mask) .* pos3ScaleFY;
ext.FYSAPOS(pos0Mask) = ext.FYSAPOS(pos0Mask) .* pos0ScaleFY;

ext.MZSAPOS = -ext.SCALEDMZ(aboveZeroSA);
ext.MZSAPOS(pos6Mask) = ext.MZSAPOS(pos6Mask) .* pos6ScaleMZ;
ext.MZSAPOS(pos3Mask) = ext.MZSAPOS(pos3Mask) .* pos3ScaleMZ;
ext.MZSAPOS(pos0Mask) = ext.MZSAPOS(pos0Mask) .* pos0ScaleMZ;

figure
tiledlayout(1,2)
nexttile
hold on
grid on
box on
plot3(ext.SA, ext.SL, ext.FY, 'ko')
plot3(ext.SA, ext.SL, ext.SCALEDFY, 'b*')
plot3(ext.MIRRORSA, ext.MIRRORSL, ext.FYSAPOS, 'r*')
legend('Original', 'Scaled R25B --> LC0' ,'Artificial LC0')
xlabel('Slip Angle [deg]')
ylabel('Longitudinal Slip (SL)')
zlabel('Lateral Force [N]')
title('FY')

nexttile
hold on
grid on
box on
plot3(ext.SA, ext.SL, ext.MZ, 'ko')
plot3(ext.SA, ext.SL, ext.SCALEDMZ, 'b*')
plot3(ext.MIRRORSA, ext.MIRRORSL, ext.MZSAPOS, 'r*')
legend('Original', 'Scaled R25B --> LC0', 'Artificial LC0')
xlabel('Slip Angle [deg]')
ylabel('Longitudinal Slip (SL)')
zlabel('Aligning Torque [N-m]')
title('MZ')

sgtitle('Artificial Force Construction')

%% CONSTRUCT ARTIFICIAL LC0 DATA

ext.CONSTRUCTEDSA = [ext.SA; ext.MIRRORSA];
ext.CONSTRUCTEDSL = [ext.SL; ext.MIRRORSL];
ext.CONSTRUCTEDFZ = [ext.FZ; ext.MIRRORFZ];
ext.CONSTRUCTEDIA = [ext.IA; ext.MIRRORIA];
ext.CONSTRUCTEDFY = [ext.SCALEDFY; ext.FYSAPOS];
ext.CONSTRUCTEDMZ = [ext.SCALEDMZ; ext.MZSAPOS];

close all
figure
tiledlayout(2,3)
nexttile
hold on
grid on
box on
plot3(ext.SA, ext.SL, ext.FY, 'bo')
plot3(ext.CONSTRUCTEDSA, ext.CONSTRUCTEDSL, ext.CONSTRUCTEDFY, 'r.')
legend('Original', 'CONSTRUCTED')
xlabel('Slip Angle [deg]')
ylabel('Longitudinal Slip (SL)')
zlabel('Lateral Force [N]')
title('FY vs. SA vs. SL')

nexttile
hold on
grid on
box on
plot3(ext.SA, ext.FZ, ext.FY, 'bo')
plot3(ext.CONSTRUCTEDSA, ext.CONSTRUCTEDFZ, ext.CONSTRUCTEDFY, 'r.')
legend('Original', 'CONSTRUCTED')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('FY vs. SA vs. FZ')

nexttile
hold on
grid on
box on
plot3(ext.SA, ext.IA, ext.FY, 'bo')
plot3(ext.CONSTRUCTEDSA, ext.CONSTRUCTEDIA, ext.CONSTRUCTEDFY, 'r.')
legend('Original', 'CONSTRUCTED')
xlabel('Slip Angle [deg]')
ylabel('Inclination Angle [deg]')
zlabel('Lateral Force [N]')
title('FY vs. IA vs. FZ')

nexttile
hold on
grid on
box on
plot3(ext.SA, ext.SL, ext.MZ, 'bo')
plot3(ext.CONSTRUCTEDSA, ext.CONSTRUCTEDSL, ext.CONSTRUCTEDMZ, 'r.')
legend('Original', 'CONSTRUCTED')
xlabel('Slip Angle [deg]')
ylabel('Longitudinal Slip (SL)')
zlabel('Aligning Torque [N-m]')
title('MZ vs. SA vs. SL')

nexttile
hold on
grid on
box on
plot3(ext.SA, ext.FZ, ext.MZ, 'bo')
plot3(ext.CONSTRUCTEDSA, ext.CONSTRUCTEDFZ, ext.CONSTRUCTEDMZ, 'r.')
legend('Original', 'CONSTRUCTED')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Aligning Torque [N-m]')
title('MZ vs. SA vs. FZ')

nexttile
hold on
grid on
box on
plot3(ext.SA, ext.SL, ext.MZ, 'bo')
plot3(ext.CONSTRUCTEDSA, ext.CONSTRUCTEDSL, ext.CONSTRUCTEDMZ, 'r.')
legend('Original', 'CONSTRUCTED')
xlabel('Slip Angle [deg]')
ylabel('Inclination Angle [deg]')
zlabel('Aligning Torque [N-m]')
title('MZ vs. SA vs. IA')

sgtitle('ORIGINAL R25B vs. CONSTRUCTED LC0')



%% SECTION BREAK

%{

             *     ,MMM8&&&.            *
                  MMMM88&&&&&    .
                 MMMM88&&&&&&&
     *           MMM88&&&&&&&&
                 MMM88&&&&&&&&
                 'MMM88&&&&&&'
                   'MMM8&&&'      *
          |\___/|
          )     (             .              '
         =\     /=
           )===(       *
          /     \
          |     |
         /       \
         \       /
  _/\_/\_/\__  _/_/\_/\_/\_/\_/\_/\_/\_/\_/\_
  |  |  |  |( (  |  |  |  |  |  |  |  |  |  |
  |  |  |  | ) ) |  |  |  |  |  |  |  |  |  |
  |  |  |  |(_(  |  |  |  |  |  |  |  |  |  |
  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |

%}


%% PACEJKA MF 5.2 LATERAL FORCE (COMBINED SLIP)

close all

% INPUTS TO MODEL = [SL, SA, FZ, IA]

INPUT = [ext.CONSTRUCTEDSL, ext.CONSTRUCTEDSA, ext.CONSTRUCTEDFZ, ext.CONSTRUCTEDIA];

% LATERAL FORCE COEFFICIENTS FIRST GUESS
RBY1 = -.461;
RBY2 = 1.196;
RBY3 = -0.015;
RBY4 = -1;
RCY1 = -1.081;
REY1 = -1;
REY2 = 1;
RHY1 = 0.009;
RHY2 = 0.1;
RVY1 = 0.053;
RVY2 = -0.073;
RVY3 = 0.517;
RVY4 = 1.44;
RVY5 = 1.9;
RVY6 = -1.71;

% LATERAL FORCE COEFFICIENTS AFTER FIRST FIT
% RBY1 = 45.2985;
% RBY2 = 2.9321;
% RBY3 = -0.54184;
% RBY4 = -577.6666;
% RCY1 = -0.8243;
% REY1 = 0.099247;
% REY2 = -0.82422;
% RHY1 = 0.0063881;
% RHY2 = 0.017405;
% RVY1 = -0.0090963;
% RVY2 = -0.03794;
% RVY3 = -0.16381;
% RVY4 = 475.6084;
% RVY5 = 2.9918;
% RVY6 = -10.3038;

clear FF RESNORM

F_str = {'RBY1' 'RBY2' 'RBY3' 'RBY4' 'RCY1' 'REY1' 'REY2' 'RHY1' ...
         'RHY2' 'RVY1' 'RVY2' 'RVY3' 'RVY4' 'RVY5' 'RVY6'};

% Keeps track of coefficients
F_old = [RBY1 RBY2 RBY3 RBY4 RCY1 REY1 REY2 RHY1 ...
         RHY2 RVY1 RVY2 RVY3 RVY4 RVY5 RVY6];

options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig1 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 COMBINED FY Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [F, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_COMBINED_FY', F_old, INPUT, ext.CONSTRUCTEDFY, [], [], options);

    FF(:, run) = F;

    for n = 1:15
        subplot(3, 5, n)
        bar(FF(n, :), 'group')
        title(['F(' num2str(n) ')' ' =' F_str{n}], 'FontSize', 8)
    end

    for n = 1:15
        disp(['F_old(' num2str(n) ') = ' num2str(F_old(n)) ';    ' 'F(' num2str(n) ') = ' num2str(F(n)) ';'])
        eval(['F_old(' num2str(n) ') = ' num2str(F(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end

%% CHECK PACEJKA FY FIT
% close all

PSS_FY_COEFFS_FOR_FIT = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 ...
                         PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 ...
                         PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 ...
                         PVY4];

PSS_INPUT = [INPUT(:, 2) INPUT(:, 3) INPUT(:, 4)];

fy_PSS_fit = Pacejka52_PSS_FY(PSS_FY_COEFFS_FOR_FIT, PSS_INPUT);

fy_fit = Pacejka52_COMBINED_FY(F, INPUT);

close all
% FY vs. SA vs. FZ
figure
tiledlayout(1, 3)
ax1 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 2), INPUT(:, 3), ext.CONSTRUCTEDFY, 'k.')
plot3(INPUT(:, 2), INPUT(:, 3), fy_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load [N]')
zlabel('Lateral Force [N]')
legend('CONSTRUCTED DATA','FITTED DATA')
title('FY vs. SA vs. FZ')

% FY vs. SA vs. SL
ax2 = nexttile;
grid on
box on
hold on
plot3(PSS_INPUT(:, 1), zeros(size(INPUT(:, 1))), fy_PSS_fit, 'b*')
plot3(INPUT(:, 2), INPUT(:, 1), ext.CONSTRUCTEDFY, 'k.')
plot3(INPUT(:, 2), INPUT(:, 1), fy_fit, 'ro')
ylabel('Longitudinal Slip')
xlabel('Slip Angle [deg]')
zlabel('Lateral Force [N]')
legend('PURE SIDE SLIP FY', 'CONSTRUCTED DATA','FITTED COMBINED FY')
title('FY vs. SA vs. SL')

% FY vs. SA vs. IA
ax3 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 2), INPUT(:, 4), ext.CONSTRUCTEDFY, 'k.')
plot3(INPUT(:, 2), INPUT(:, 4), fy_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('IA [deg]')
zlabel('Lateral Force [N]')
legend('CONSTRUCTED DATA','FITTED DATA')
title('FY vs. SA vs. IA')

%% ACCEPT COMBINED SLIP LATERAL FORCE COEFFS.

RBY1 = F(1);
RBY2 = F(2);
RBY3 = F(3);
RBY4 = F(4);
RCY1 = F(5);
REY1 = F(6);
REY2 = F(7);
RHY1 = F(8);
RHY2 = F(9);
RVY1 = F(10);
RVY2 = F(11);
RVY3 = F(12);
RVY4 = F(13);
RVY5 = F(14);
RVY6 = F(15);

%% PACEJKA MF 5.2 ALIGNING TORQUE (COMBINED SLIP)

close all

% INPUTS TO MODEL = [SL, SA, FZ, IA]

INPUT = [ext.CONSTRUCTEDSL, ext.CONSTRUCTEDSA, ext.CONSTRUCTEDFZ, ext.CONSTRUCTEDIA];

% ALIGNING TORQUE COEFFICIENTS FIRST GUESS FROM TEXT
% SSZ1 = 0.043;
% SSZ2 = 0.001;
% SSZ3 = 0.731;
% SSZ4 = -0.238;

% ALIGNING TORQUE COEFFS. AFTER FIRST FIT
SSZ1 = -0.026661;
SSZ2 = -0.037927;
SSZ3 = 0.41917;
SSZ4 = -0.54008;

clear GG RESNORM

G_str = {'SSZ1' 'SSZ2' 'SSZ3' 'SSZ4'};

% Keeps track of coefficients
G_old = [SSZ1 SSZ2 SSZ3 SSZ4];

options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig1 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 COMBINED FY Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [G, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_COMBINED_MZ', G_old, INPUT, ext.CONSTRUCTEDMZ, [], [], options);

    GG(:, run) = G;

    for n = 1:4
        subplot(1, 4, n)
        bar(GG(n, :), 'group')
        title(['G(' num2str(n) ')' ' =' G_str{n}], 'FontSize', 8)
    end

    for n = 1:4
        disp(['G_old(' num2str(n) ') = ' num2str(G_old(n)) ';    ' 'G(' num2str(n) ') = ' num2str(G(n)) ';'])
        eval(['G_old(' num2str(n) ') = ' num2str(G(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end

%% CHECK PACEJKA MZ FIT
% close all

PSS_MZ_COEFFS_FOR_FIT = [QBZ1 QBZ2 QBZ3 QBZ5 QBZ6 QBZ9 QBZ10...
                         QCZ1 ...
                         QDZ1 QDZ2 QDZ3 QDZ4 QDZ6 QDZ7 QDZ8 QDZ9 QDZ10 QDZ11 ...
                         QEZ1 QEZ2 QEZ3 QEZ4 QEZ5 ...
                         QHZ1 QHZ2 QHZ3 QHZ4];

PSS_INPUT = [INPUT(:, 2) INPUT(:, 3) INPUT(:, 4)];

mz_PSS_fit = Pacejka52_PSS_MZ(PSS_MZ_COEFFS_FOR_FIT, PSS_INPUT);

mz_fit = Pacejka52_COMBINED_MZ(G, INPUT);

close all
% MZ vs. SA vs. FZ
figure
tiledlayout(1, 3)
ax1 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 2), INPUT(:, 3), ext.CONSTRUCTEDMZ, 'k.')
plot3(INPUT(:, 2), INPUT(:, 3), mz_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load [N]')
zlabel('Aligning Torque [N-m]')
legend('CONSTRUCTED DATA','FITTED DATA')
title('MZ vs. SA vs. FZ')

% MZ vs. SA vs. SL
ax2 = nexttile;
grid on
box on
hold on
plot3(PSS_INPUT(:, 1), zeros(size(INPUT(:, 1))), mz_PSS_fit, 'b*')
plot3(INPUT(:, 2), INPUT(:, 1), ext.CONSTRUCTEDMZ, 'k.')
plot3(INPUT(:, 2), INPUT(:, 1), mz_fit, 'ro')
ylabel('Longitudinal Slip')
xlabel('Slip Angle [deg]')
zlabel('Aligning Torque [N-m]')
legend('PURE SIDE SLIP MZ', 'CONSTRUCTED DATA','FITTED COMBINED MZ')
title('MZ vs. SA vs. SL')

% MZ vs. SA vs. IA
ax3 = nexttile;
grid on
box on
hold on
plot3(INPUT(:, 2), INPUT(:, 4), ext.CONSTRUCTEDMZ, 'k.')
plot3(INPUT(:, 2), INPUT(:, 4), mz_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('IA [deg]')
zlabel('Aligning Torque [N-m]')
legend('CONSTRUCTED DATA','FITTED DATA')
title('MZ vs. SA vs. IA')

%% ACCEPT COMBINED SLIP ALIGNING TORQUE COEFFS

SSZ1 = G(1);
SSZ2 = G(2);
SSZ3 = G(3);
SSZ4 = G(4);

%% SECTION BREAK













%% HOOSIER 16x7.5-10 LC0 - FY, MX, MZ - 8PSI
load B1965run16.mat

%% Overview Plot 
close all

kPa2psi = 0.145038;
kmh2mps = 0.277778;
V = V*kmh2mps;
P = P*kPa2psi;

figure
tiledlayout(7,1)

ax1 = nexttile;
plot(ET, P)
ylabel('Inflation Pressure [psi]')
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(ET, IA)
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile;
plot(ET, FZ)
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(ET, SL)
ylabel('Longitudinal Slip (SL)')
xlabel('Elapsed Time [s]')

ax5 = nexttile;
plot(ET, SA)
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

ax6 = nexttile;
plot(ET, FX)
ylabel('FX [N]')
xlabel('Elapsed Time [s]')

ax7 = nexttile;
plot(ET, V)
ylabel('V [m/s]')
xlabel('Elapsed Time [s]')

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7], 'x')

sgtitle('Overview')
%% EXTRACT RELEVANT SECTION

% Evaluating tire at 8 PSI

close all

idx     = ET > 130 & ET < 650;
EText   = ET(idx);
SAext   = SA(idx);
FZext   = FZ(idx);
FYext   = FY(idx);
MZext   = MZ(idx);
MXext   = MX(idx);
RLext   = RL(idx);
IAext   = IA(idx);
Pext    = P(idx);

tiledlayout(2,2) 
ax1 = nexttile; 
plot(EText,Pext) 
ylabel('Inflation Pressure [psi]') 
xlabel('Elapsed Time [s]')

ax2 = nexttile;
plot(EText,IAext) 
ylabel('Inclination Angle [deg]')
xlabel('Elapsed Time [s]')

ax3 = nexttile; 
plot(EText,FZext) 
ylabel('Normal Load [N]')
xlabel('Elapsed Time [s]')

ax4 = nexttile;
plot(EText,SAext) 
ylabel('Slip Angle [deg]')
xlabel('Elapsed Time [s]')

sgtitle('EXTRACTED DATA')

linkaxes([ax1 ax2 ax3 ax4], 'x')

figure
plot3(SAext, FZext, FYext, 'ko')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('EXTRACTED DATA')

%% 'POLISH' DATA
close all

% Fit a spline using all points and find the zeros
m = 1:length(SAext);
sp = spline(m, SAext);
z = fnzeros(sp);
z = round(z(1, :));
z = [1, z, length(SAext)]; % hardcode in the first and last zeros

% Check the zeros
figure
hold on
plot(m, SAext)
plot(z, zeros(length(z)), 'ko')
yline(0)
xlabel('Data Point')
ylabel('SA [deg]')
title('SA Points for Segmentation')

% New data structure row counter
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

%% POLISH COMPARISON
close all;

tiledlayout(1, 2)
ax1 = nexttile;
plot3(SAext, FZext, FYext, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('ORIGINAL')

ax2 = nexttile;
plot3(ext.SA, ext.FZ, ext.FY, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Lateral Force [N]')
title('NEW')

linkprop([ax1 ax2], 'CameraPosition');
sgtitle('PURE SIDE SLIP FY COMPARISON')

%% CHECK PURE SIDE SLIP MX DATA
close all

tiledlayout(1, 2)
ax1 = nexttile;
hold on
grid, box on
plot3(SAext, FZext, MXext, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Overturning Moment [N-m]')
title('ORIGINAL')

ax2 = nexttile;
hold on
grid, box on
plot3(ext.SA, ext.FZ, ext.MX, 'o')
xlabel('Slip Angle [deg]')
ylabel('Normal Load [N]')
zlabel('Overturning Moment [N-m]')
title('NEW')

linkprop([ax1 ax2], 'CameraPosition');
sgtitle('PURE SIDE SLIP MX COMPARISON')

%% PACEJKA MF 5.2 OVERTURNING MOMENT

% ADHERE TO ADAPTED SAE
FZO = 1100; % N, nominal load
RO = 0.2032; % m, unloaded tire radius

% INPUTS TO MODEL = [SA, FZ, IA]

INPUT = [zeros(size(ext.SA)) ext.SA, ext.FZ, ext.IA];

% OVERTURNING MOMENT COEFFICIENTS INITIAL GUESSES, FROM TEXT
QSX1        = 0;
QSX2        = 0;
QSX3        = 0;

% OVERTURNING MOMENT COEFFICIENTS (EQ 4.122-124) INITIAL GUESSES, FROM TEXT PG. 206
% QX1 = 0.042;
% QX2 = 0.56;
% QX3 = 0.955;
% QX4 = 2.35;
% QX5 = 1.25;
% QX6 = 0.46;

clear CC RESNORM

C_str = {'QSX1' 'QSX2' 'QSX3'};
% C_str = {'QX1' 'QX2' 'QX3' 'QX4' 'QX5' 'QX6'};

% Keeps track of coefficients
C_old = [QSX1 QSX2 QSX3];
% C_old = [QX1 QX2 QX3 QX4 QX5 QX6];


options = optimset('MaxFunEvals',20000,'MaxIter',20000,'Display','final','TolX',1e-7,'TolFun',1e-7);

fig3 = figure('MenuBar', 'none', 'Name', 'Pacejka MF 5.2 MX Fitting Result', 'NumberTitle', 'off');

% COEFFS. DETERMINATION
for run = 1:20

    [C, RESNORM(run), RESIDUAL, EXITFLAG] = lsqcurvefit('Pacejka52_COMBINED_MX', C_old, INPUT, ext.MX, [], [], options);

    CC(:, run) = C;

    for n = 1:3
        subplot(1, 3, n)
        bar(CC(n, :), 'group')
        title(['C(' num2str(n) ')' ' =' C_str{n}], 'FontSize', 8)
    end

    for n = 1:3
        disp(['C_old(' num2str(n) ') = ' num2str(C_old(n)) ';    ' 'C(' num2str(n) ') = ' num2str(C(n)) ';'])
        eval(['C_old(' num2str(n) ') = ' num2str(C(n)) ' -1*eps*rand;'])
    end

    disp(['ITERATION: ' num2str(run) '      RESNORM: ' num2str(RESNORM(run))]);
    drawnow
end



%% CHECK PACEJKA PURE SIDE SLIP MZ FIT
close all

% MX vs. SA vs. FZ
figure
mx_fit = Pacejka52_COMBINED_MX(C, INPUT);
hold on
box on
grid on
plot3(INPUT(:, 2),INPUT(:, 3), ext.MX, 'k.')
plot3(INPUT(:, 2),INPUT(:, 3), mx_fit, 'ro')
xlabel('Slip Angle [deg]')
ylabel('Vertical Load (N)')
zlabel('Overturning Moment [N-m]')
legend('Data Pts','FITTED DATA')
title('MX vs. SA vs. FZ, 0 SL')


%% ACCEPT OVERTURNING MOMENT COEFFS.

QSX1        = C(1);
QSX2        = C(2);
QSX3        = C(3);

%% ALL DONE!!

%{

                      /^--^\     /^--^\     /^--^\
                      \____/     \____/     \____/
                     /      \   /      \   /      \
KAT                 |        | |        | |        |
                     \__  __/   \__  __/   \__  __/
|^|^|^|^|^|^|^|^|^|^|^|^\ \^|^|^|^/ /^|^|^|^|^\ \^|^|^|^|^|^|^|^|^|^|^|^|
| | | | | | | | | | | | |\ \| | |/ /| | | | | | \ \ | | | | | | | | | | |
########################/ /######\ \###########/ /#######################
| | | | | | | | | | | | \/| | | | \/| | | | | |\/ | | | | | | | | | | | |
|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|


                             ＿＿
　　　　　   ＞　 フ
　　　　　| 　_　 _l
　 　　　／` ミ＿xノ
　　 　 /　　　 　 |
　　　 /　 ヽ　　 ﾉ
　 　 │　　|　|　|
　／￣|　　 |　|　|
　| (￣ヽ＿_ヽ_)__)
　＼二つ

   ____
  (.   \
    \  |  
     \ |___(\--/)
   __/    (  . . )
  "'._.    '-.O.'
       '-.  \ "|\
          '.,,/'.,,mrf

%}
