clc;
clear;
close all;
%{

    [A;         =       [UPPER FORE;
     B;                  UPPER AFT;
     C;                  LOWER FORE;
     D;                  LOWER AFT;
     E;                  PUSH ROD;
     F]                  TIE ROD;   ]

%}

mm2m = 0.001;

inboard.A = [102.857,   -265.562,   -285] .* mm2m;
inboard.B = [-245.283,  -247.422,   -296.648] .* mm2m;
inboard.C = [110,       -186.312,   -115] .* mm2m;
inboard.D = [-177.051,  -196.511,   -144.279] .* mm2m;
inboard.E = [-219.244,  -178.214,   -515.125] .* mm2m;
inboard.F = [76,        -182.88,    -141] .* mm2m;

outboard.A = [-5.823,   -528.913,   -287.976] .* mm2m;
outboard.B = outboard.A;
outboard.C = [1.398,    -553.661,   -99.994] .* mm2m;
outboard.D = outboard.C;
outboard.E = [-45.431,  -464.716,   -317.706] .* mm2m;
outboard.F = [76,       -552.779,   -129] .* mm2m;

outboard.CONTACTPATCH = [0,        -600,        0] .* mm2m;

% inboard.A = [-1380.224,   -286.29,    -225.015] .* mm2m;
% inboard.B = [-1575,       -255,       -262] .* mm2m;
% inboard.C = [-1328.191,   -333,       -114.555] .* mm2m;
% inboard.D = [-1571.989,   -204.693,   -107.055] .* mm2m;
% inboard.E = [-1331.337,   -223.854,   -397.858] .* mm2m;
% inboard.F = [-1600.5,     -203,       -133] .* mm2m;
% 
% outboard.A = [-1533,      -508,       -297] .* mm2m;
% outboard.B = outboard.A;
% outboard.C = [-1488,      -522,       -108] .* mm2m;
% outboard.D = outboard.C;
% outboard.E = [-1477.071,  -488.745,   -131.616] .* mm2m;
% outboard.F = [-1615,      -531,       -141] .* mm2m;
% 
% outboard.CONTACTPATCH = [-1535,      -600,       0] .* mm2m;
% outboard.CALIPER = [-67.952, -580.5, -188.018] .* mm2m;
% outboard.HUB = [0, -600, -200] .* mm2m;

% figure
% hold on
% grid, box on
% 
% % SET SAE AXIS SYSTEM
% set(gca, 'ZDir', 'reverse')
% set(gca, 'XDir', 'reverse')
% axis equal
% 
% plot3(inboard.A(1), inboard.A(2), inboard.A(3), 'ro')
% plot3(inboard.B(1), inboard.B(2), inboard.B(3), 'go')
% plot3(inboard.C(1), inboard.C(2), inboard.C(3), 'bo')
% plot3(inboard.D(1), inboard.D(2), inboard.D(3), 'mo')
% plot3(inboard.E(1), inboard.E(2), inboard.E(3), 'yo')
% plot3(inboard.F(1), inboard.F(2), inboard.F(3), 'ko')
% 
% plot3(outboard.A(1), outboard.A(2), outboard.A(3), 'r*')
% plot3(outboard.B(1), outboard.B(2), outboard.B(3), 'g*')
% plot3(outboard.C(1), outboard.C(2), outboard.C(3), 'b*')
% plot3(outboard.D(1), outboard.D(2), outboard.D(3), 'm*')
% plot3(outboard.E(1), outboard.E(2), outboard.E(3), 'y*')
% plot3(outboard.F(1), outboard.F(2), outboard.F(3), 'k*')
% 
% line([inboard.A(1) outboard.A(1)], [inboard.A(2) outboard.A(2)], [inboard.A(3) outboard.A(3)])
% line([inboard.B(1) outboard.B(1)], [inboard.B(2) outboard.B(2)], [inboard.B(3) outboard.B(3)])
% line([inboard.C(1) outboard.C(1)], [inboard.C(2) outboard.C(2)], [inboard.C(3) outboard.C(3)])
% line([inboard.D(1) outboard.D(1)], [inboard.D(2) outboard.D(2)], [inboard.D(3) outboard.D(3)])
% line([inboard.E(1) outboard.E(1)], [inboard.E(2) outboard.E(2)], [inboard.E(3) outboard.E(3)])
% line([inboard.F(1) outboard.F(1)], [inboard.F(2) outboard.F(2)], [inboard.F(3) outboard.F(3)])
% 
% title('FRONT A ARM WIREFRAME')

%% 

% FIND LENGTHS PER ARM
L.A = norm(outboard.A - inboard.A);
L.B = norm(outboard.B - inboard.B);
L.C = norm(outboard.C - inboard.C);
L.D = norm(outboard.D - inboard.D);
L.E = norm(outboard.E - inboard.E);
L.F = norm(outboard.F - inboard.F);

% FIND FORCE DIRECTION PER ARM
dir.A = (outboard.A - inboard.A) ./ L.A;
dir.B = (outboard.B - inboard.B) ./ L.B;
dir.C = (outboard.C - inboard.C) ./ L.C;
dir.D = (outboard.D - inboard.D) ./ L.D;
dir.E = (outboard.E - inboard.E) ./ L.E;
dir.F = (outboard.F - inboard.F) ./ L.F;

% FIND DISTANCE FROM O (AXIS ORIGIN) TO ARM
r.A = inboard.A;
r.B = inboard.B;
r.C = inboard.C;
r.D = inboard.D;
r.E = inboard.E;
r.F = inboard.F;
r.CONTACTPATCH = outboard.CONTACTPATCH;
% r.CALIPER = outboard.CALIPER - outboard.CONTACTPATCH;
% r.HUB = outboard.HUB - outboard.CONTACTPATCH;

% FIND DISTANCE CROSS FORCE DIR.
mDir.A = cross(r.A, dir.A);
mDir.B = cross(r.B, dir.B);
mDir.C = cross(r.C, dir.C);
mDir.D = cross(r.D, dir.D);
mDir.E = cross(r.E, dir.E);
mDir.F = cross(r.F, dir.F);


%%
FXtire = 1500;
FYtire = 0;
FZtire = -200;

M_overturning = 0;
M_torque = 0;
M_aligning = 0;

Mtire = cross(r.CONTACTPATCH, [FXtire, FYtire, FZtire]);

% FXbrake = 0;
% FYbrake = 0;
% FZbrake = 0;
% 
% Mcaliper = cross(r.CALIPER, [FXbrake, FYbrake, FZbrake]);
% Mhub = cross(r.HUB, [-FXbrake, -FYbrake, -FZbrake]);
% Mhub = [0, 0, 0];



A = [dir.A(1)   dir.B(1)    dir.C(1)    dir.D(1)    dir.E(1)    dir.F(1);
     dir.A(2)   dir.B(2)    dir.C(2)    dir.D(2)    dir.E(2)    dir.F(2);
     dir.A(3)   dir.B(3)    dir.C(3)    dir.D(3)    dir.E(3)    dir.F(3);
     mDir.A(1)  mDir.B(1)   mDir.C(1)   mDir.D(1)   mDir.E(1)   mDir.F(1);
     mDir.A(2)  mDir.B(2)   mDir.C(2)   mDir.D(2)   mDir.E(2)   mDir.F(2);
     mDir.A(3)  mDir.B(3)   mDir.C(3)   mDir.D(3)   mDir.E(3)   mDir.F(3)];

b = [-(FXtire);
     -(FYtire);
     -(FZtire);
     -(M_overturning + Mtire(1));
     -(M_torque      + Mtire(2));
     -(M_aligning    + Mtire(3))];

% TENSION IS NEGATIVE AT THIS LINE
FORCES = A\b;

% CHANGE TENSION TO POSITIVE AND COMPRESSION TO NEGATIVE
FORCES = -FORCES;

disp('FRONT TIRE LOADS (SAE)');
disp(['FX                 : ' num2str(FXtire) ' N']);
disp(['FY                 : ' num2str(FYtire) ' N']);
disp(['FZ                 : ' num2str(FZtire) ' N']);
disp(['OVERTURNING MOMENT : ' num2str(M_overturning) ' N-m']);
disp(['TORQUE             : ' num2str(M_torque) ' N-m']);
disp(['ALIGNING TORQUE    : ' num2str(M_aligning) ' N-m']);

fprintf('\n');

disp('A ARM FORCES')
disp(['FRONT UPPER FORE : ' num2str(FORCES(1)) ' N']);
disp(['FRONT UPPER AFT  : ' num2str(FORCES(2)) ' N']);
disp(['FRONT LOWER FORE : ' num2str(FORCES(3)) ' N']);
disp(['FRONT LOWER AFT  : ' num2str(FORCES(4)) ' N']);
disp(['FRONT PUSH ROD   : ' num2str(FORCES(5)) ' N']);
disp(['FRONT TIE ROD    : ' num2str(FORCES(6)) ' N']);




