function n = FY_EXTRAPOLATION_COEFF(ALPHA)
    % MAPPING RANGE
    x = [10 11 20];
    y = [1 0.99 0.5];
    
    coeffs = polyfit(x, y, 2);

    fcn = @(slip) coeffs(1).*(slip.^2) + coeffs(2).*(slip.^1) + coeffs(3);

    n = fcn(abs(ALPHA));

end

