function tireForces = getTireForces(INPUT)
    %% PULL INPUTS

    % GETS TIRE FORCES DURING PURE SLIP CONDITIONS

    % INPUT = [SL, SA, FZ, IA], [DEGREES] IN SAE AXIS SYSTEM
    % FZ = NEGATIVE
    % COLUMN VECTOR
    
    % CONVENTION: 
    % SAE FOR TTC TIRE AXIS SYSTEM AND INPUT
    % ADAPTED SAE FOR PACEJKA MODEL
    % SAE --> ASAE TAKEN CARE OF PER FXN
    
    % !!!!! SEE TTC TIRE AXIS SYSTEM.png FOR SA DIRECTION !!!!!
    % !!!!! SEE PACEJKA AXIS SYSTEM PT1 & 2.png FOR SA DIRECTION !!!!!!
    
    % KAPPA = LONGITUDINAL SLIP, ALPHA = SA, FZ = NORMAL LOAD, GAMMA = IA 
    
    % ALL ANGLES USED IN MODEL PARAM. DETERMINATION = RADIAN

    KAPPA   = INPUT(:, 1);
    ALPHA   = INPUT(:, 2);
    FZ      = INPUT(:, 3);
    GAMMA   = INPUT(:, 4);

    % CREATE INPUT FOR PURE SIDE SLIP FY, MZ
    INPUT_PSS = [ALPHA, FZ, GAMMA];

    % CREATE INPUT FOR PURE LONG. SLIP FX
    INPUT_PLS = [KAPPA, FZ, GAMMA];

    % CREATE INPUT FOR COMBINED SLIP
    INPUT_CS = [KAPPA, ALPHA, FZ, GAMMA];

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


    %% GET PURE SLIP COEFFS

    % Load PSS LAT. COEFFS.
    A = load('PSS LATERAL FORCE COEFFS.mat').A;
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

    % REDUNDANT, FOR READABILITY
    PSS_FY_COEFFS = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 ...
                     PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 ...
                     PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 ...
                     PVY4];


    % LOAD PSS MZ COEFFS
    B = load('PSS ALIGNING TORQUE COEFFS.mat').B;
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

    % REDUNDANT, FOR READABILITY
    PSS_MZ_COEFFS = [QBZ1 QBZ2 QBZ3  QBZ5  QBZ6 QBZ9 QBZ10 ...
                     QCZ1 QDZ1 QDZ2  QDZ3  QDZ4 QDZ6 QDZ7 ...
                     QDZ8 QDZ9 QDZ10 QDZ11 QEZ1 QEZ2 QEZ3 ...
                     QEZ4 QEZ5 QHZ1  QHZ2  QHZ3 QHZ4];


    % LOAD PLS LONG. COEFFS
    D = load('PLS LONGITUDINAL FORCE COEFFS.mat').D;
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

    % REDUNDANT
    PLS_FX_COEFFS = [PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 ...
                     PKX1 PKX2 PKX3 PHX1 PHX2 PVX1 PVX2];



    %% GET COMBINED SLIP COEFFS

    % LOAD COMBINED FX COEFFS.
    E = load('COMBINED LONGITUDINAL FORCE COEFFS.mat').E;
    RBX1 = E(1);
    RBX2 = E(2);
    RBX3 = E(3);
    RCX1 = E(4);
    REX1 = E(5);
    REX2 = E(6);
    RHX1 = E(7);

    % REDUNDANT
    COMBINED_FX_COEFFS = [RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1];



    % LOAD COMBINED FY COEFFS.
    F = load('COMBINED LATERAL FORCE COEFFS.mat').F;
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

    % REDUNDANT
    COMBINED_FY_COEFFS = [RBY1 RBY2 RBY3 RBY4 RCY1 REY1 REY2 RHY1 ...
                          RHY2 RVY1 RVY2 RVY3 RVY4 RVY5 RVY6];



    % LOAD COMBINED MZ COEFFS.
    G = load('COMBINED ALIGNING TORQUE COEFFS.mat').G;
    SSZ1 = G(1);
    SSZ2 = G(2);
    SSZ3 = G(3);
    SSZ4 = G(4);

    % REDUNDANT
    COMBINED_MZ_COEFFS = [SSZ1 SSZ2 SSZ3 SSZ4];



    C = load('COMBINED OVERTURNING MOMENT COEFFS.mat').C;
    QSX1 = C(1);
    QSX2 = C(2);
    QSX3 = C(3);

    % REDUNDANT
    COMBINED_MX_COEFFS = [QSX1 QSX2 QSX3];


    %% GET FORCES

    % RUN FUNCTIONS
    PSS_FY = Pacejka52_PSS_FY(PSS_FY_COEFFS, INPUT_PSS);
    PLS_FX = Pacejka52_PLS_FX(PLS_FX_COEFFS, INPUT_PLS);
    PSS_MZ = Pacejka52_PSS_MZ(PSS_MZ_COEFFS, INPUT_PSS);

    COMBINED_FY = Pacejka52_COMBINED_FY(COMBINED_FY_COEFFS, INPUT_CS);
    COMBINED_FX = Pacejka52_COMBINED_FX(COMBINED_FX_COEFFS, INPUT_CS);
    COMBINED_MZ = Pacejka52_COMBINED_MZ(COMBINED_MZ_COEFFS, INPUT_CS);
    COMBINED_MX = Pacejka52_COMBINED_MX(COMBINED_MX_COEFFS, INPUT_CS);

    %% APPLY SCALING FACTORS TO EXTRAPOLATION DATA
    KAPPA_EXP_INDS = abs(KAPPA) >= 0.2;

    FX_EXTRAP_COEFFS = FX_EXTRAPOLATION_COEFF(KAPPA(KAPPA_EXP_INDS));
    COMBINED_FX(KAPPA_EXP_INDS) = COMBINED_FX(KAPPA_EXP_INDS) .* FX_EXTRAP_COEFFS;

    ALPHA_EXP_INDS = abs(ALPHA) >= 10;
    FY_EXTRAP_COEFFS = FY_EXTRAPOLATION_COEFF(ALPHA(ALPHA_EXP_INDS));
    COMBINED_FY(ALPHA_EXP_INDS) = COMBINED_FY(ALPHA_EXP_INDS) .* FY_EXTRAP_COEFFS;

    %% COMPILE INTO STRUCTURE
    tireForces.PURE.FY = PSS_FY;
    tireForces.PURE.FX = PLS_FX;
    tireForces.PURE.MZ = PSS_MZ;

    % PURE STRUCT IS NOT REALLY NEEDED SINCE COMBINED IS CALCULATED FROM
    % PURE DATA, MEANING (COMBINED FY @ SL = 0) == (PURE FY)
    tireForces.COMBINED.FY = COMBINED_FY;
    tireForces.COMBINED.FX = COMBINED_FX;
    tireForces.COMBINED.MZ = COMBINED_MZ;
    tireForces.COMBINED.MX = COMBINED_MX;
end