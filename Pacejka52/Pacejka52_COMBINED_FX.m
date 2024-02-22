function FX = Pacejka52_COMBINED_FX(R, INPUT)
    
    % COMBINED LONG. FORCES

    global FZO

    global LFZO 

    global PCX1 ...
           PDX1 PDX2 PDX3 ...
           PEX1 PEX2 PEX3 PEX4 ...
           PKX1 PKX2 PKX3 ...
           PHX1 PHX2 ...
           PVX1 PVX2

    global LXALPHA 

    RBX1 = R(1);
    RBX2 = R(2);
    RBX3 = R(3);
    RCX1 = R(4);
    REX1 = R(5);
    REX2 = R(6);
    RHX1 = R(7);

    PLS_FX_COEFFS = [PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 ...
                     PKX1 PKX2 PKX3 PHX1 PHX2 PVX1 PVX2];

    % BEFORE AXIS TRANSFORMATION AND UNIT CONVERSION
    PLS_INPUT = [INPUT(:, 1) INPUT(:, 3) INPUT(:, 4)];
    FXO = Pacejka52_PLS_FX(PLS_FX_COEFFS, PLS_INPUT);

    % Redundant, but to make a point, SAE --> ASAE
    FXO = FXO;

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

    EXALPHA = REX1 + REX2 .* DFZ;
    
    SHXALPHA = RHX1;
    
    ALPHAS = ALPHA_AST + SHXALPHA;
    
    BXALPHA = (RBX1 + RBX3 .* GAMMA_AST .^ 2) .* cos(atan(RBX2 .* KAPPA)) .* LXALPHA;
    
    BXALPHA = abs(BXALPHA); % APPLY >0 CONSTRAINT
    
    CXALPHA = RCX1;

    GXALPHAO = cos(CXALPHA .* atan(BXALPHA .* SHXALPHA - EXALPHA .* (BXALPHA .* SHXALPHA - atan(BXALPHA .* SHXALPHA))));
    
    GXALPHA = cos(CXALPHA .* atan(BXALPHA .* ALPHAS - EXALPHA .* (BXALPHA .* ALPHAS - atan(BXALPHA .* ALPHAS)))) ./ GXALPHAO;
    
    GXALPHA = abs(GXALPHA); % APPLY >0 CONSTRAINT
    
    FX = GXALPHA .* FXO;

    % COMBINED FX <= PURE FX
    if (abs(FX) > abs(FXO))
        FX = FXO;
    end

    % Redundant, but to make a point, ASAE --> SAE
    FX = FX;

end

