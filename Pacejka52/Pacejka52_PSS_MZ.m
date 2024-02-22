function MZ = Pacejka52_PSS_MZ(Q, INPUT)

    % ALIGNING TORQUE (PURE SIDE SLIP)
    
    global FZO RO VCX

    global PCY1 ...
           PDY1 PDY2 PDY3 ...
           PEY1 PEY2 PEY3 PEY4 PEY5...
           PKY1 PKY2 PKY3 PKY4 PKY5 PKY6 PKY7...
           PHY1 PHY2 ...
           PVY1 PVY2 PVY3 PVY4;

    global LFZO LKYALPHA LCY...
           LHY LVY LKYGAMMA LKZGAMMA ...
           LT LMR LMUYAST LMUYPRIME

    global ZETA0 ZETA2 ZETA3 ZETA4 ZETA5 ZETA6 ZETA7 ZETA8

    global EPSILON

    PSS_FY_COEFFS = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 ...
                     PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 ...
                     PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 ...
                     PVY4];

    % !!! BEFORE CHANGING AXIS SYSTEM !!!
    FYO = Pacejka52_PSS_FY(PSS_FY_COEFFS, INPUT);

    % Pacejka52_PSS_FY outputs in SAE
    % SAE --> ASAE
    FYO = -FYO;

    QBZ1    = Q(1);
    QBZ2    = Q(2);
    QBZ3    = Q(3);
    QBZ5    = Q(4);
    QBZ6    = Q(5);
    QBZ9    = Q(6);
    QBZ10   = Q(7);
    QCZ1    = Q(8);
    QDZ1    = Q(9);
    QDZ2    = Q(10);
    QDZ3    = Q(11);
    QDZ4    = Q(12);
    QDZ6    = Q(13);
    QDZ7    = Q(14);
    QDZ8    = Q(15);
    QDZ9    = Q(16);
    QDZ10   = Q(17);
    QDZ11   = Q(18);
    QEZ1    = Q(19);
    QEZ2    = Q(20);
    QEZ3    = Q(21);
    QEZ4    = Q(22);
    QEZ5    = Q(23);  
    QHZ1    = Q(24);
    QHZ2    = Q(25);
    QHZ3    = Q(26);
    QHZ4    = Q(27);

    % ADHERE TO ADAPTED SAE
    ALPHA   = -1.*deg2rad(INPUT(:, 1));
    FZ      = abs(INPUT(:, 2));
    GAMMA   = deg2rad(INPUT(:, 3));
    FZOPRIME = FZO .* LFZO;

    ALPHA_ZERO_IND = ALPHA == 0;

    ALPHA_AST = tan(ALPHA);
    GAMMA_AST = sin(GAMMA);

    DFZ = (FZ - FZOPRIME) ./ FZOPRIME;

    SHT = QHZ1 + QHZ2 .* DFZ + (QHZ3 + QHZ4 .* DFZ) .* GAMMA_AST;

    ALPHAT = ALPHA_AST + SHT;

    DTO = FZ .* (RO ./ FZOPRIME) .* (QDZ1 + QDZ2 .* DFZ) .* LT .* sign(VCX);

    DT = DTO .* (1 + QDZ3 .* abs(GAMMA_AST) + QDZ4 .* GAMMA_AST .^ 2) .* ZETA5;

    CT = QCZ1;

    BT = (QBZ1 + QBZ2 .* DFZ + QBZ3 .* DFZ .^ 2) .* (1 + QBZ5 .* abs(GAMMA_AST) + QBZ6 .* GAMMA_AST .^ 2) .* LKYALPHA ./ LMUYAST;

    ET = (QEZ1 + QEZ2 .* DFZ + QEZ3 .* DFZ .^ 2) .* (1 + (QEZ4 + QEZ5 .* GAMMA_AST) .* (2./pi) .* atan(BT .* CT .* ALPHAT));

    TO = DT .* cos(CT .* atan(BT .* ALPHAT - ET .* (BT .* ALPHAT - atan(BT .* ALPHAT)))) .* cos(ALPHA);

    MZOPRIME = (-TO) .* FYO;

    DR = FZ .* RO .* ((QDZ6 + QDZ7 .* DFZ) .* LMR .* ZETA2 + (QDZ8 + QDZ9 .* DFZ) .* GAMMA_AST .* LKZGAMMA .* ZETA0 + (QDZ10 + QDZ11 .* DFZ) .* GAMMA_AST .* abs(GAMMA_AST) .* ZETA0) .* cos(ALPHA) .* LMUYAST .* sign(VCX) + ZETA8 - 1;

    CR = ZETA7;

    
    % ----- FROM Pacejka52_PSS_FY -----

    MUY = abs(((PDY1 + PDY2 .* DFZ) ./ (1 + PDY3 .* GAMMA_AST .^ 2)) .* LMUYAST);
    DY = MUY .* FZ .* ZETA2;
    CY = PCY1 .* LCY;
    KYALPHA = PKY1 .* FZOPRIME .* sin(PKY4 .* atan(FZ ./ ((PKY2 + PKY5 .* (GAMMA_AST .^ 2)) .* FZOPRIME))) ./ (1 + PKY3 .* GAMMA_AST .^ 2) .* ZETA3 .* LKYALPHA;
    BY = KYALPHA ./ (CY .* DY + EPSILON .* sign(KYALPHA));
    KYGAMMAO = FZ .* (PKY6 + PKY7 .* DFZ) .* LKYGAMMA; % CAMBER STIFFNESS
    SVYGAMMA = FZ .* (PVY3 + PVY4 .* DFZ) .* GAMMA_AST .* LKYGAMMA .* LMUYPRIME .* ZETA2;
    SHY = (PHY1 + PHY2 .* DFZ) .* LHY + (KYGAMMAO .* GAMMA_AST - SVYGAMMA) .* ZETA4 ./ (KYALPHA + EPSILON .* sign((KYGAMMAO .* GAMMA_AST - SVYGAMMA))) + ZETA3 - 1;
    SVY = FZ .* (PVY1 + PVY2 .* DFZ) .* LVY .* LMUYPRIME .* ZETA2 + SVYGAMMA;

    % -----------------------------

    BR = (QBZ9 .* LKYALPHA ./ LMUYAST + QBZ10 .* BY .* CY) .* ZETA6;

    KYALPHAPRIME = KYALPHA + EPSILON;

    SHF = SHY + SVY ./ KYALPHAPRIME;

    ALPHAR = ALPHA_AST + SHF;

    MZRO = DR .* cos(CR .* atan(BR .* ALPHAR));

    MZO = MZOPRIME + MZRO;

    % ADAPTED SAE --> SAE FOR LSQCURVEFIT
    MZ = -MZO;

    MZ(ALPHA_ZERO_IND) = 0;


    % ----- ADDITIONAL TERMS -----

    KZALPHAO = DTO .* KYALPHA; % ALIGNING TORQUE STIFFNESS, if KYALPHA = KYALPHA @ GAMMA = 0
    KZGAMMAO = FZ .* RO .* (QDZ8 + QDZ9 .* DFZ) .* LKZGAMMA - DTO .* KYGAMMAO; % CAMBER STIFFNESS FOR ALIGNING TORQUE

    % ----------------------------
end

