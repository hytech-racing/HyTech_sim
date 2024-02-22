function MZ = Pacejka52_COMBINED_MZ(S, INPUT)
    
    global FZO RO VCX

    global LFZO LKXKAPPA LKYALPHA LCY...
           LHY LVY LKYGAMMA LKZGAMMA ...
           LT LMR LMUYAST LMUYPRIME 

    % PURE FY PARAMETERS
    global PCY1 ...
           PDY1 PDY2 PDY3 ...
           PKY1 PKY2 PKY3 PKY4 PKY5 PKY6 PKY7...
           PHY1 PHY2 ...
           PVY1 PVY2 PVY3 PVY4

    % PURE FX PARAMETERS
    global PKX1 PKX2 PKX3 ...

    % PURE MZ PARAMETERS
    global QBZ1 QBZ2 QBZ3 QBZ5 QBZ6 QBZ9 QBZ10...
           QCZ1 ...
           QDZ1 QDZ2 QDZ3 QDZ4 QDZ6 QDZ7 QDZ8 QDZ9 QDZ10 QDZ11 ...
           QEZ1 QEZ2 QEZ3 QEZ4 QEZ5 ...
           QHZ1 QHZ2 QHZ3 QHZ4...

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

    global LVYKAPPA LS

    global ZETA0 ZETA2 ZETA3 ZETA4 ZETA5 ZETA6 ZETA7 ZETA8

    global EPSILON

    SSZ1 = S(1);
    SSZ2 = S(2);
    SSZ3 = S(3);
    SSZ4 = S(4);

    COMBINED_FXCOEFFS = [RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1];
    COMBINED_FYCOEFFS = [RBY1 RBY2 RBY3 RBY4 RCY1 REY1 REY2 RHY1 ...
                         RHY2 RVY1 RVY2 RVY3 RVY4 RVY5 RVY6];

    FX = Pacejka52_COMBINED_FX(COMBINED_FXCOEFFS, INPUT);
    FY = Pacejka52_COMBINED_FY(COMBINED_FYCOEFFS, INPUT);

    % SAE --> ASAE
    FX = FX;
    FY = -FY;

    % ADHERE TO ADAPTED SAE
    KAPPA = INPUT(:, 1) ;
    ALPHA   = -1.*deg2rad(INPUT(:, 2));
    FZ = abs(INPUT(:, 3));
    GAMMA = deg2rad(INPUT(:, 4));

    FZOPRIME = FZO .* LFZO;
    DFZ = (FZ - FZOPRIME) ./ FZOPRIME;
    GAMMA_AST = sin(GAMMA);
    ALPHA_AST = tan(ALPHA);


    %% FROM Pacejka52_PSS_FY.m

    MUY = ((PDY1 + PDY2 .* DFZ) ./ (1 + PDY3 .* GAMMA_AST .^ 2)) .* LMUYPRIME;

    MUY = abs(MUY); % APPLY >0 CONSTRAINT

    KYALPHA = PKY1 .* FZOPRIME .* sin(PKY4 .* atan(FZ ./ ((PKY2 + PKY5 .* (GAMMA_AST .^ 2)) .* FZOPRIME))) ./ (1 + PKY3 .* GAMMA_AST .^ 2) .* ZETA3 .* LKYALPHA;

    SVYGAMMA = FZ .* (PVY3 + PVY4 .* DFZ) .* GAMMA_AST .* LKYGAMMA .* LMUYPRIME .* ZETA2;

    KYGAMMAO = FZ .* (PKY6 + PKY7 .* DFZ) .* LKYGAMMA; % CAMBER STIFFNESS

    SHY = (PHY1 + PHY2 .* DFZ) .* LHY + (KYGAMMAO .* GAMMA_AST - SVYGAMMA) .* ZETA4 ./ (KYALPHA + EPSILON .* sign((KYGAMMAO .* GAMMA_AST - SVYGAMMA))) + ZETA3 - 1;

    SVY = FZ .* (PVY1 + PVY2 .* DFZ) .* LVY .* LMUYPRIME .* ZETA2 + SVYGAMMA;

    CY = PCY1 .* LCY;

    DY = MUY .* FZ .* ZETA2;

    BY = KYALPHA ./ (CY .* DY + EPSILON .* sign(KYALPHA));

    % -----------------------

    %% FROM Pacejka52_PLS_FX.m

    KXKAPPA = FZ .* (PKX1 + PKX2 .* DFZ) .* exp(PKX3 .* DFZ) .* LKXKAPPA; % LONGITUDINAL SLIP STIFFNESS


    % -----------------------

    %% FROM Pacejka52_PSS_MZ.m

    BT = (QBZ1 + QBZ2 .* DFZ + QBZ3 .* DFZ .^ 2) .* (1 + QBZ5 .* abs(GAMMA_AST) + QBZ6 .* GAMMA_AST .^ 2) .* LKYALPHA ./ LMUYAST;

    CT = QCZ1;

    DTO = FZ .* (RO ./ FZOPRIME) .* (QDZ1 + QDZ2 .* DFZ) .* LT .* sign(VCX);

    DT = DTO .* (1 + QDZ3 .* abs(GAMMA_AST) + QDZ4 .* GAMMA_AST .^ 2) .* ZETA5;

    SHT = QHZ1 + QHZ2 .* DFZ + (QHZ3 + QHZ4 .* DFZ) .* GAMMA_AST;

    ALPHAT = ALPHA_AST + SHT;

    KYALPHAPRIME = KYALPHA + EPSILON;

    SHF = SHY + SVY ./ KYALPHAPRIME;

    ALPHAR = ALPHA_AST + SHF;

    DR = FZ .* RO .* ((QDZ6 + QDZ7 .* DFZ) .* LMR .* ZETA2 + (QDZ8 + QDZ9 .* DFZ) .* GAMMA_AST .* LKZGAMMA .* ZETA0 + (QDZ10 + QDZ11 .* DFZ) .* GAMMA_AST .* abs(GAMMA_AST) .* ZETA0) .* cos(ALPHA) .* LMUYAST .* sign(VCX) + ZETA8 - 1;

    CR = ZETA7;

    BR = (QBZ9 .* LKYALPHA ./ LMUYAST + QBZ10 .* BY .* CY) .* ZETA6;

    ET = (QEZ1 + QEZ2 .* DFZ + QEZ3 .* DFZ .^ 2) .* (1 + (QEZ4 + QEZ5 .* GAMMA_AST) .* (2./pi) .* atan(BT .* CT .* ALPHAT));

    % -----------------------

    %% FROM Pacejka_COMBINED_FY.m

    DVYKAPPA = MUY .* FZ .* (RVY1 + RVY2 .* DFZ + RVY3 .* GAMMA_AST) .* cos(atan(RVY4 .* ALPHA_AST)) .* ZETA2;

    SVYKAPPA = DVYKAPPA .* sin(RVY5 .* atan(RVY6 .* KAPPA)) .* LVYKAPPA;

    % ---------------------------

    %% COMBINED MZ EQUATIONS
    
    ALPHATEQ = sqrt(ALPHAT .^ 2 + ((KXKAPPA ./ KYALPHAPRIME) .^ 2) .* (KAPPA .^ 2)) .* sign(ALPHAT);

    ALPHAREQ = sqrt(ALPHAR .^ 2 + ((KXKAPPA ./ KYALPHAPRIME) .^ 2) .* (KAPPA .^ 2)) .* sign(ALPHAR);

    T = DT .* cos(CT .* atan(BT .* ALPHATEQ - ET .* (BT .* ALPHATEQ - atan(BT .* ALPHATEQ)))) .* cos(ALPHA);

    FYPRIME = FY - SVYKAPPA;

    MZPRIME = -T .* FYPRIME;

    MZR = DR .* cos(CR .* atan(BR .* ALPHAREQ));

    S = RO .* (SSZ1 + SSZ2 .* (FY ./ FZOPRIME) + (SSZ3 + SSZ4 .* DFZ) .* GAMMA_AST) .* LS;

    MZ = MZPRIME + MZR + S .* FX;

    % ASAE --> SAE
    MZ = -MZ;

end

