function FY = Pacejka52_PSS_FY(P, INPUT)
    
    % LATERAL FORCE (PURE SIDE SLIP)

    global FZO

    global LFZO LKYALPHA LCY...
           LEY LHY LVY LKYGAMMA ...
           LMUYAST LMUYPRIME 

    global ZETA2 ZETA3 ZETA4

    global EPSILON

    PCY1 = P(1);
    PDY1 = P(2);
    PDY2 = P(3);
    PDY3 = P(4);
    PEY1 = P(5);
    PEY2 = P(6);
    PEY3 = P(7);
    PEY4 = P(8);
    PEY5 = P(9);
    PKY1 = P(10);
    PKY2 = P(11);
    PKY3 = P(12);
    PKY4 = P(13);
    PKY5 = P(14);
    PKY6 = P(15);
    PKY7 = P(16);
    PHY1 = P(17);
    PHY2 = P(18);
    PVY1 = P(19);
    PVY2 = P(20);
    PVY3 = P(21);
    PVY4 = P(22);

    % ADHERE TO ADAPTED SAE
    ALPHA   = -1.*deg2rad(INPUT(:, 1));
    FZ      = abs(INPUT(:, 2));
    GAMMA   = deg2rad(INPUT(:, 3));
    FZOPRIME = FZO .* LFZO;

    ALPHA_ZERO_IND = ALPHA == 0;

    DFZ = (FZ - FZOPRIME) ./ FZOPRIME;

    GAMMA_AST = sin(GAMMA);

    CY = PCY1 .* LCY;
    
    MUY = abs(((PDY1 + PDY2 .* DFZ) ./ (1 + PDY3 .* GAMMA_AST .^ 2)) .* LMUYAST);

    DY = MUY .* FZ .* ZETA2;

    % IF GAMMA = 0, KYALPHA = CORNERING STIFFNESS
    % KYALPHA = PKY1 .* FZO .* sin(2.0 .* atan(FZ ./ (PKY2 .* FZOPRIME))) ./ (1 - PKY3 .* GAMMA_AST) .* ZETA3 .* LKYALPHA;
    KYALPHA = PKY1 .* FZOPRIME .* sin(PKY4 .* atan(FZ ./ ((PKY2 + PKY5 .* (GAMMA_AST .^ 2)) .* FZOPRIME))) ./ (1 + PKY3 .* GAMMA_AST .^ 2) .* ZETA3 .* LKYALPHA;

    SVYGAMMA = FZ .* (PVY3 + PVY4 .* DFZ) .* GAMMA_AST .* LKYGAMMA .* LMUYPRIME .* ZETA2;

    KYGAMMAO = FZ .* (PKY6 + PKY7 .* DFZ) .* LKYGAMMA; % CAMBER STIFFNESS

    SHY = (PHY1 + PHY2 .* DFZ) .* LHY + (KYGAMMAO .* GAMMA_AST - SVYGAMMA) .* ZETA4 ./ (KYALPHA + EPSILON .* sign((KYGAMMAO .* GAMMA_AST - SVYGAMMA))) + ZETA3 - 1;
    
    ALPHA_AST = tan(ALPHA);

    ALPHAY = ALPHA_AST + SHY;
    
    BY = KYALPHA ./ (CY .* DY + EPSILON .* sign(KYALPHA));
    
    % FORMULA FROM TTC
    % EY = (PEY1 + PEY2 .* DFZ) .* (1 - (PEY3 + PEY4 .* GAMMA_AST) .* sign(ALPHAY)) .* LEY;

    % FORMULA FROM TEXT
    EY = (PEY1 + PEY2 .* DFZ) .* (1 + PEY5 .* GAMMA_AST .^ 2 - (PEY3 + PEY4 .* GAMMA_AST) .* sign(ALPHAY)) .* LEY;
    
    SVY = FZ .* (PVY1 + PVY2 .* DFZ) .* LVY .* LMUYPRIME .* ZETA2 + SVYGAMMA;
    
    FYO = DY .* sin(CY .* atan(BY .* ALPHAY - EY .* (BY .* ALPHAY - atan(BY .* ALPHAY)))) + SVY;

    % ADAPTED SAE --> SAE FOR LSQCURVEFIT
    FY = -FYO;

    FY(ALPHA_ZERO_IND) = 0;

end


