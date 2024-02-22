function MX = Pacejka52_COMBINED_MX(Q, INPUT)
    
    % OVERTURNING MOMENT
    
    global FZO RO

    % global PCY1 ...
    %        PDY1 PDY2 PDY3 ...
    %        PEY1 PEY2 PEY3 PEY4 PEY5...
    %        PKY1 PKY2 PKY3 PKY4 PKY5 PKY6 PKY7...
    %        PHY1 PHY2 ...
    %        PVY1 PVY2 PVY3 PVY4;

    global RBY1 RBY2 RBY3 RBY4 ...
           RCY1 ...
           REY1 REY2 ...
           RHY1 RHY2 ...
           RVY1 RVY2 RVY3 RVY4 RVY5 RVY6

    global LFZO 

    global LMX

    QSX1 = Q(1);
    QSX2 = Q(2);
    QSX3 = Q(3);

    % QX1 = Q(1);
    % QX2 = Q(2);
    % QX3 = Q(3);
    % QX4 = Q(4);
    % QX5 = Q(5);
    % QX6 = Q(6);

    % PSS_FY_COEFFS = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 ...
    %              PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 ...
    %              PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 ...
    %              PVY4];

    COMBINED_FY_COEFFS = [RBY1 RBY2 RBY3 RBY4 ...
                          RCY1 ...
                          REY1 REY2 ...
                          RHY1 RHY2 ...
                          RVY1 RVY2 RVY3 RVY4 RVY5 RVY6];

    FY = Pacejka52_COMBINED_FY(COMBINED_FY_COEFFS, INPUT);

    % FYO = Pacejka52_PSS_FY(PSS_FY_COEFFS, INPUT);

    % Pacejka52_PSS_FY outputs in SAE
    % SAE --> ASAE
    % FYO = -FYO;
    FY = -FY;

    % ADHERE TO ADAPTED SAE
    ALPHA   = -1.*deg2rad(INPUT(:, 2));
    FZ      = abs(INPUT(:, 3));
    GAMMA   = deg2rad(INPUT(:, 4));
    FZOPRIME = FZO .* LFZO;

    ALPHA_AST = tan(ALPHA);
    GAMMA_AST = sin(GAMMA);

    DFZ = (FZ - FZOPRIME) ./ FZOPRIME;

    MX = FZ .* RO .* (QSX1 - QSX2 .* GAMMA_AST + QSX3 .* FY ./ FZOPRIME) .* LMX;

    % Y1 = RO .* QX1 .* (FY ./ FZO);
    % 
    % Y2 = -RO .* QX2 .* cos(QX3 .* atan(QX4 .* FZ ./ FZO) .^ 2) .* sin(QX5 .* atan(QX6 .* FY ./ FZO));
    % 
    % MX = -FZ .* (Y1 + Y2);

    % ADAPTED SAE --> SAE FOR LSQCURVEFIT
    MX = -MX;

end

