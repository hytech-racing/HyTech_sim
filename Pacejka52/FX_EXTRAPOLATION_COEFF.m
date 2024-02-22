function n = FX_EXTRAPOLATION_COEFF(KAPPA)
    
    % MAPPING RANGE
    x = [0.2 0.3 0.5 0.8 1];
    y = [1 0.9 0.7 0.55 0.5];

    coeffs = polyfit(x, y, 3);

    fcn = @(slip) coeffs(1).*(slip.^3) + coeffs(2).*(slip.^2) + coeffs(3).*slip + coeffs(4);

    n = fcn(abs(KAPPA));

end

