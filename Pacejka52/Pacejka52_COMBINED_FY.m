function FY = Pacejka52_COMBINED_FY(R, INPUT)
    % COMBINED LONG. FORCES

    global FZO 

    global LFZO ...
           LMUYPRIME 

    global PCY1 ...
           PDY1 PDY2 PDY3 ...
           PEY1 PEY2 PEY3 PEY4 PEY5...
           PKY1 PKY2 PKY3 PKY4 PKY5 PKY6 PKY7...
           PHY1 PHY2 ...
           PVY1 PVY2 PVY3 PVY4

    global LYKAPPA LVYKAPPA 

    global ZETA2 

    RBY1 = R(1);
    RBY2 = R(2);
    RBY3 = R(3);
    RBY4 = R(4);
    RCY1 = R(5);
    REY1 = R(6);
    REY2 = R(7);
    RHY1 = R(8);
    RHY2 = R(9);
    RVY1 = R(10);
    RVY2 = R(11);
    RVY3 = R(12);
    RVY4 = R(13);
    RVY5 = R(14);
    RVY6 = R(15);

    PSS_FY_COEFFS = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 ...
                     PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 ...
                     PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 ...
                     PVY4];

    % BEFORE AXIS TRANSFORMATION AND UNIT CONVERSION
    PSS_INPUT = [INPUT(:, 2) INPUT(:, 3) INPUT(:, 4)];
    FYO = Pacejka52_PSS_FY(PSS_FY_COEFFS, PSS_INPUT);

    % SAE --> ASAE
    FYO = -FYO;

    % ADHERE TO ADAPTED SAE
    KAPPA = INPUT(:, 1) ;
    ALPHA   = -1.*deg2rad(INPUT(:, 2));
    FZ = abs(INPUT(:, 3));
    GAMMA = deg2rad(INPUT(:, 4));

    FZOPRIME = FZO .* LFZO;
    DFZ = (FZ - FZOPRIME) ./ FZOPRIME;
    GAMMA_AST = sin(GAMMA);
    ALPHA_AST = tan(ALPHA);
    
    % EQUATIONS

    MUY = ((PDY1 + PDY2 .* DFZ) ./ (1 + PDY3 .* GAMMA_AST .^ 2)) .* LMUYPRIME;

    MUY = abs(MUY); % APPLY >0 CONSTRAINT

    DVYKAPPA = MUY .* FZ .* (RVY1 + RVY2 .* DFZ + RVY3 .* GAMMA_AST) .* cos(atan(RVY4 .* ALPHA_AST)) .* ZETA2;

    SVYKAPPA = DVYKAPPA .* sin(RVY5 .* atan(RVY6 .* KAPPA)) .* LVYKAPPA;

    EYKAPPA = REY1 + REY2 .* DFZ;

    CYKAPPA = RCY1;

    BYKAPPA = (RBY1 + RBY4 .* GAMMA_AST .^ 2) .* cos(atan(RBY2 .* (ALPHA_AST - RBY3))) .* LYKAPPA;

    BYKAPPA = abs(BYKAPPA); % APPLY >0 CONSTRAINT

    SHYKAPPA = RHY1 + RHY2 .* DFZ;

    KAPPAS = KAPPA + SHYKAPPA;

    GYKAPPAO = cos(CYKAPPA .* atan(BYKAPPA .* SHYKAPPA - EYKAPPA .* (BYKAPPA .* SHYKAPPA - atan(BYKAPPA .* SHYKAPPA))));

    GYKAPPA = cos(CYKAPPA .* atan(BYKAPPA .* KAPPAS - EYKAPPA .* (BYKAPPA .* KAPPAS - atan(BYKAPPA .* KAPPAS)))) ./ GYKAPPAO;

    GYKAPPA = abs(GYKAPPA); % APPLY >0 CONSTRAINT

    FY = GYKAPPA .* FYO + SVYKAPPA;

    % COMBINED FY <= PURE FY
    if (abs(FY) > abs(FYO))
        FY = FYO;
    end

    % ASAE --> SAE
    FY = -FY;

end

