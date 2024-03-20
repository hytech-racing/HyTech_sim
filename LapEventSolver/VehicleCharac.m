function veh = VehicleCharac(dispGraph)

    %% VEHICLE PARAMS
    m = 250; % kg
    g = 9.81; % m/s2
    fz_per_tire_static = m / 4 * g; % N
    fz_tot = m * g; % N
    
    wb = 1.535; % m, wheelbase
    b = 0.78; % m, CG to rear axle
    a = wb - b; % m, CG to front axle
    frontalA = 1;
    
    rho = 1.293; 
    Cl = 3.71;
    Cd = 1.23;
    
    %% TIRE MODEL
    
    R_tire = 0.2; % m, tire radius
    c_roll = 0.1; % Rolling resistance
    
    tireFactor_X = .6;
    tireFactor_Y = .6;
    
    FZO = 1100; % N, Tire nominal load used to create the pacejka fits
    
    load 'PSS LATERAL FORCE COEFFS.mat'
    
    PDY1 = A(2);
    PDY2 = A(3);
    % PVY1 = A(19);
    % PVY2 = A(20);
    
    MUY = @(FZ) (PDY1 + PDY2 .* ((FZ - FZO) ./ FZO));
    % SVY = @(FZ) FZ .* (PVY1 + PVY2 .* ((FZ - FZO) ./ FZO));
    FY = @(FZ) abs((MUY(FZ) .* FZ) .* tireFactor_Y); 
    
    load 'PLS LONGITUDINAL FORCE COEFFS.mat'
    
    PDX1 = D(2);
    PDX2 = D(3);
    PVX1 = D(14);
    PVX2 = D(15);
    
    MUX = @(FZ) (PDX1 + PDX2 .* ((FZ - FZO) ./ FZO));
    SVX = @(FZ) FZ .* (PVX1 + PVX2 .* ((FZ - FZO) ./ FZO));
    FX = @(FZ) (MUX(FZ) .* FZ + SVX(FZ)) .* tireFactor_X;
        
    % Cornering stiffness
    SA = linspace(-2, 2)';
    
    INPUT = [zeros(size(SA)) SA -fz_per_tire_static .* ones(size(SA)) zeros(size(SA))];
    tireForces = getTireForces(INPUT);
    
    CF = polyfit(SA, tireForces.PURE.FY, 1);
    
    CF = abs(CF(1)); % N/deg, Cornering stiffness front, 1 tire
    CR = CF; % N/deg, Cornering stiffness rear, 1 tire
    
    %% STEERING
    
    corneringMatrix = 2 .* [CF       -CF + CR;
                            a*CF     -a*CF + b*CR];
    
    %% MOTOR
    GR = 11.86; % Gear ratio
    motorOmegaLimit = 2094; % rad/s, motor angular velocity limit
    wheelOmegaLimit = motorOmegaLimit ./ GR;
    vel_max = wheelOmegaLimit * R_tire; % m/s, maximum linear velocity
    powerLimit = 80; % kW
    powerLimit = powerLimit * 1000; % Nm/s
    
    angVelVec = linspace(0, motorOmegaLimit, 50);
    totalPowerLimitedMotorTorque = powerLimit./angVelVec;
    vehicle_speed = angVelVec./GR.*R_tire;
    
    %% GGV
    
    vel_vec = vehicle_speed';
    
    if (vel_vec(end) ~= vel_max)
        vel_vec = [vel_vec; vel_max];
    end
    
    N = 45;
    GGV = zeros(length(vel_vec), 2*N-1, 3);
    
    maxTireTractiveForceVec = zeros(length(vel_vec), 1);
    
    for i = 1:length(vel_vec)
        aero_downF = 0.5 * rho * Cl * frontalA * vel_vec(i) ^ 2;
        aero_dragF = -0.5 * rho * Cd * frontalA * vel_vec(i) ^ 2;
    
        roll_F = -c_roll * (fz_tot + aero_downF);
    
        fz_tire = (fz_tot + aero_downF) / 4;
    
        ax_drag = (aero_dragF + roll_F) / m;
    
        ay_max = FY(fz_tire) .* 4 ./ m;
    
        ax_tire_max_accel = FX(fz_tire) .* 4 ./ m;
    
        maxTireTractiveForceVec(i) = ax_tire_max_accel * m;
    
        ax_tire_max_decel = -ax_tire_max_accel;
    
        ax_power_limit = interp1(vehicle_speed, totalPowerLimitedMotorTorque, vel_vec(i)) * GR / R_tire / m;
        
        ax_power_limit = ax_power_limit * ones(N, 1);
    
        ay = ay_max * cosd(linspace(0, 180, N))';
    
        ax_tire_accel = ax_tire_max_accel * sqrt(1 - (ay / ay_max) .^ 2);
    
        ax_accel = min(ax_tire_accel, ax_power_limit) + ax_drag;
    
        ax_dec = ax_tire_max_decel * sqrt(1 - (ay / ay_max) .^ 2) + ax_drag;
    
        GGV(i, :, 1) = [ax_accel', ax_dec(2:end)'];
        GGV(i, :, 2) = [ay', flipud(ay(2:end))'];
        GGV(i, :, 3) = vel_vec(i) .* ones(1, 2*N-1);
    end
    
    if (dispGraph)
    
        figure
        tiledlayout(1, 2)
        nexttile
        surf(GGV(:, :, 2), GGV(:, :, 1), GGV(:, :, 3))
        xlabel('Lat. Accel. [m/s^2]')
        ylabel('Long. Accel. [m/s^2]')
        zlabel('Speed [m/s]')
        title('GGV Diagram')
        
        nexttile
        hold on
        box on
        grid on
        yyaxis left
        plot(vehicle_speed, totalPowerLimitedMotorTorque' .* GR ./ R_tire)
        plot(vehicle_speed, maxTireTractiveForceVec)
        ylabel('Force [N]')
        yyaxis right
        plot(vehicle_speed, min(totalPowerLimitedMotorTorque' .* GR ./ R_tire, maxTireTractiveForceVec))
        ylabel('Force [N]')
        xlabel('Speed [m/s]')
        legend('Power Limited Total Motor Tractive Force', 'Total Tire Tractive Force', 'Limited Tractive Force')
        title('Traction Model')
    
    end
    
    veh.m = m;
    veh.g = g;
    veh.fz_per_tire_static = fz_per_tire_static;
    veh.fz_tot = fz_tot;
    veh.wb = wb;
    veh.a = a;
    veh.b = b;
    veh.frontalA = frontalA;
    veh.rho = rho;
    veh.Cl = Cl;
    veh.Cd = Cd;
    veh.R_tire = R_tire;
    veh.c_roll = c_roll;
    veh.tireFactor_Y = tireFactor_Y;
    veh.tireFactor_X = tireFactor_X;
    veh.PDY1 = PDY1;
    veh.PDY2 = PDY2;
    veh.FZO = FZO;
    veh.FYfunc = FY;
    veh.FXfunc = FX;
    veh.corneringMatrix = corneringMatrix;
    veh.vel_max = vel_max;
    veh.GR = GR;
    veh.powerLimit.vehicle_speed = vehicle_speed;
    veh.powerLimit.totalPowerLimitedMotorTorque = totalPowerLimitedMotorTorque;
    veh.powerLimit.maxTireTractiveForceVec = maxTireTractiveForceVec;
    veh.GGV = GGV;
    
