function [FX] = Pacejka52_PLS_FX(A, INPUT)
    
    % LONGITUDINAL FORCE (PURE LONGITUDINAL SLIP)

    global FZO

    global LFZO LMUX LKXKAPPA LCX ...
           LEX LHX LVX ...
           LMUXPRIME

    global ZETA1

    global EPSILON

    PCX1 = A(1);
    PDX1 = A(2);
    PDX2 = A(3);
    PDX3 = A(4);
    PEX1 = A(5);
    PEX2 = A(6);
    PEX3 = A(7);
    PEX4 = A(8);
    PKX1 = A(9);
    PKX2 = A(10);
    PKX3 = A(11);
    PHX1 = A(12);
    PHX2 = A(13);
    PVX1 = A(14);
    PVX2 = A(15);

    % ADHERE TO ADAPTED SAE
    KAPPA = INPUT(:,1) ;
    FZ = abs(INPUT(:,2));
    GAMMA = deg2rad(INPUT(:,3));

    FZOPRIME = FZO .* LFZO;
    DFZ = (FZ - FZOPRIME) ./ FZOPRIME;
    GAMMA_AST = sin(GAMMA);

    KAPPA_ZERO_IND = KAPPA == 0;
    

    SHX = (PHX1 + PHX2 .* DFZ) .* LHX;

    % SIMPLIFICATION SINCE EPSILON = 0;
    SVX = FZ .* (PVX1 + PVX2 .* DFZ) .* LVX .* LMUXPRIME .* ZETA1;

    KAPPAX = KAPPA + SHX;

    CX = PCX1 .* LCX;

    % DIFFERENT FORMULA FROM TTC, INCLUDES GAMMA_AST
    MUX = (PDX1 + PDX2 .* DFZ) .* (1 - PDX3 .* GAMMA_AST .^ 2) .* LMUX;

    DX = MUX .* FZ .* ZETA1;

    KXKAPPA = FZ .* (PKX1 + PKX2 .* DFZ) .* exp(PKX3 .* DFZ) .* LKXKAPPA; % LONGITUDINAL SLIP STIFFNESS

    BX = KXKAPPA ./ (CX .* DX + EPSILON);

    EX = (PEX1 + PEX2 .* DFZ + PEX3 .* DFZ .^ 2) .* (1 - PEX4 .* sign(KAPPAX)) .* LEX;

    FXO = DX .* sin(CX .* atan(BX .* KAPPAX - EX .* (BX .* KAPPAX - atan(BX .* KAPPAX)))) + SVX;

    % ADAPTED SAE --> SAE, NO SIGN CHANGE
    FX = FXO;

    FX(KAPPA_ZERO_IND) = 0;
end

