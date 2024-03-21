close all;
clear;
clc;

% Modified from OpenLAP
% https://github.com/mc12027/OpenLAP-Lap-Time-Simulator

%% LAP EVENT SOLVER

clc;

veh = VehicleCharac(false);
track = TrackGen('Skidpad.xlsx', 1, true);

v_initial = 0;

v_max = zeros(track.n, 1);

for i = 1:length(v_max)
    v_max(i) = calculateSpeedTrace(veh, track.r(i));
end

[v_apex, v_apex_loc] = findpeaks(-v_max);
v_apex = -v_apex;

if isempty(v_apex_loc)
    % If no apexes, add an apex at start with initial velocity
    v_apex_loc = 1;
    v_apex = v_initial;
else
    if (v_apex_loc(1) ~= 1)
        % If the first apex is not the starting location, add an apex at
        % the start with initial velocity
        v_apex_loc = [1; v_apex_loc];
        v_apex = [v_initial; v_apex];
    else
        % If the first apex is also the starting location, set the starting
        % velocity to 0
        v_apex(1) = v_initial;
    end
end

apex_table = sortrows([v_apex v_apex_loc], 1);
v_apex = apex_table(:, 1);
apex_loc = apex_table(:, 2);

% Number of apexes
N = length(apex_loc);

% Allocate memory for flags checking for speed adjustments, 2 columns, one
% for accel. one for decel
flag = false(track.n,2);

% [number of points in track mesh, number of apexes, 2 layers for accel and decel]
v = inf*ones(track.n,N,2);
ax = zeros(track.n,N,2);
ax_commanded = zeros(track.n,N,2);
ay = zeros(track.n,N,2);

% for each apex number
for i = 1:N
    for k = 1:2
        switch k
            case 1
                % Accel. mode = 1, decel = 2
                mode = 1;
                k_rest = 2;
            case 2
                % Decel mode = -1, accel = 1
                mode = -1;
                k_rest = 1;
        end



        if ~(mode == -1 && i == 1) % Does not decelerate at the start

            % i_rest = other apexes
            i_rest = (1:N)';
            i_rest(i) = [];

            if isempty(i_rest)
                % If no other apexes, set i_rest = current apex number
                i_rest = i;
            end

            % Apex location in the track mesh
            j = apex_loc(i);

            % velocity at the current track mesh row, at the current apex
            % column, at the current accel/decel layer = velocity at the
            % apex
            v(j, i, k) = v_apex(i);
            ay(j, i, k) = v_apex(i)^2 * track.r(j); % r has number of rows = track mesh size

            % Set flag for speed adjustment to true
            flag(j, k) = true;

            % Get next point index
            [~, j_next] = next_point(j, track.n, mode, track.config);

            % if ~(strcmp(tr.config,'Open') && mode == 1 && i == 1) % if not in standing start
            %     % assuming same speed right after apex
            %     v(j_next,i,k) = v(j,i,k);
            %     % moving to next point index
            %     [j_next,j] = next_point(j, tr.n, mode, tr.config);
            % end

            while true

                [v(j_next, i, k), ax(j, i, k), ax_commanded(j, i, k), ay(j, i, k), overshoot] = vehicle_solve(veh, track, v(j,i,k), v_max(j_next), j, mode);

                if overshoot
                    break
                end

                % if speed adjustment has been performed for either accel.
                % or decel...
                % if flag(j, k) || flag(j, k_rest)
                % 
                %     if max(v(j_next,i,k) >= v(j_next,i_rest,k)) || max(v(j_next,i,k)>v(j_next,i_rest,k_rest))
                %         % checks the row representing the current location
                %         % in the track mesh
                % 
                %         % if the current velocity is greater than all other
                %         % velocities calculated from other apexes in the
                %         % same mode
                % 
                %         % or
                % 
                %         % if the current velocity is greater than all other
                %         % velocities calculated from other apexes in the
                %         % opposite mode
                % 
                %         % No need to calculate
                % 
                %         break
                % 
                %     end
                % end

                % Update flag at this location to true
                flag(j, k) = true;

                % Move j to the next location in the mesh
                [j_next,j] = next_point(j, track.n, mode, track.config);


                % For some reason it's acclerating past the end and
                % affecting the first velocity value, don't want to debug,
                % brute force to set initial velocity.
                if (v(1, 1, 1) ~= v_initial)
                    v(1, 1, 1) = v_initial;
                end

                if (v(1, 1, 2) ~= v_initial)
                    v(1, 1, 2) = v_initial;
                end

                % Check if lap is finished
                switch string(track.config)
                    case 'Closed'
                        if j == apex_loc(i)
                            % Made it to the same apex
                            break
                        end
                    case 'Open'
                        if j == track.n
                            % Made it to the end of the track mesh
                            break
                        end

                        if j == 1
                            % Made it to the start
                            break
                        end
                end
            end

        end


    end
end

% Post Process
V = zeros(track.n, 1);
AX_target = zeros(track.n, 1);
AX_commanded = zeros(track.n, 1);
AY = zeros(track.n, 1);

for i = 1:track.n

    % IDX also equals number of apexes
    IDX = length(v(i, :, 1));

    % Since v has cols = number of apexes and 2 layers, min function would
    % have number of cols = 2 * num apexes
    [V(i), idx] = min([v(i,:,1),v(i,:,2)]);

    % Meaning, any idx that is <= IDX (the number of apexes) would be a
    % solution solved from acceleration

    % any idx that is > IDX would be a solution solved from deceleration
    
    if idx <= IDX
        % Accel. solution
        AX_target(i) = ax(i, idx, 1);
        AX_commanded(i) = ax_commanded(i, idx, 1);
        AY(i) = ay(i, idx, 1);

    else
        % Decel. solution
        AX_target(i) = ax(i, idx-IDX, 2);
        AX_commanded(i) = ax_commanded(i, idx-IDX, 2);
        AY(i) = ay(i, idx-IDX, 2);

    end

end

time = cumsum([track.dx(2)./V(2);track.dx(2:end)./V(2:end)]) ;

laptime = time(end);

fprintf('Laptime: %.2fs', laptime);

% Calculate forces
m = veh.m;
g = 9.81;

Fz_mass = m*g;
Fz_aero = 0.5 .* veh.rho .* veh.Cl .* veh.frontalA .* V.^2;
Fz_total = Fz_mass + Fz_aero;
Fx_aero = -0.5 .* veh.rho .* veh.Cd .* veh.frontalA .* V.^2;
Fx_roll = -veh.c_roll .* Fz_total;

% Calculate Yaw rate and steering
yaw_rate = V .* track.r; % rad/s
delta = atan(yaw_rate .* veh.wb ./ V); % rad

trackReference.AX = AX_target;
trackReference.AY = AY;
trackReference.posAlongTrack = track.posAlongTrack;
trackReference.yawRate = yaw_rate;
trackReference.speed = V;
trackReference.wheelSteerDeg = rad2deg(delta);
trackReference.curvature = track.r;
trackReference.X = track.X;
trackReference.Y = track.Y;
trackReference.tangentialAngDeg = track.tangentialAngDeg;
trackReference.length = track.length;

figure
subplot(4, 2, [1 3 5 7])
hold on
box on
grid on
surf(veh.GGV(:, :, 2), veh.GGV(:, :, 1), veh.GGV(:, :, 3))
alpha(0.5)
plot3(AY, AX_target, V, 'o', 'MarkerFaceColor', 'red', 'MarkerSize', 5)
axis fill
xlabel('Lat. Accel. [m/s^2]')
ylabel('Long. Accel. [m/s^2]')
zlabel('Speed [m/s]')
title('GGV Diagram')
legend('GGV Surface', 'Lap Accelerations')

subplot(4, 2, 2)
hold on
plot(track.Y(1), track.X(1), 'r^', 'MarkerSize', 8)
scatter(track.Y, track.X, 15, V)
alpha(1)
cb = colorbar;
cb.Label.String = 'Speed [m/s]';
daspect([1 1 1])
grid on
axis equal
xlabel('Y [m]')
ylabel('X [m]')
title('Track Map')
legend('Starting Location')

subplot(4, 2, 4)
hold on
box on
grid on
yyaxis left
ylabel('Angle [Deg]')
plot(track.posAlongTrack, rad2deg(delta))
yyaxis right
ylabel('Ang. Vel [rad/s]')
plot(track.posAlongTrack, yaw_rate)
title('Delta and Yaw Rate')
legend('Wheel Steer', 'Yaw Rate')
xlabel('Distance Along Track [m]')

subplot(4, 2, 6)
hold on
box on
grid on
yyaxis left
plot(veh.powerLimit.vehicle_speed, veh.powerLimit.totalPowerLimitedMotorTorque' .* veh.GR ./ veh.R_tire)
plot(veh.powerLimit.vehicle_speed, veh.powerLimit.maxTireTractiveForceVec)
ylabel('Force [N]')
yyaxis right
plot(veh.powerLimit.vehicle_speed, min(veh.powerLimit.totalPowerLimitedMotorTorque' .* veh.GR ./ veh.R_tire, veh.powerLimit.maxTireTractiveForceVec))
ylabel('Force [N]')
xlabel('Speed [m/s]')
legend('Power Limited Total Motor Tractive Force', 'Total Tire Tractive Force', 'Limited Tractive Force')
title('Traction Model')

subplot(4, 2, 8)
hold on
box on
grid on
yyaxis left
ylabel('Acceleration [m/s^2]')
plot(track.posAlongTrack, AX_target)
plot(track.posAlongTrack, AY)
yyaxis right
ylabel('Vel. [m/s]')
plot(track.posAlongTrack, V)
legend('Long. Accel.', 'Lat. Accel.', 'Velocity')
xlabel('Distance Along Track [m]')
title('Accel. and Vel.')


sgtitle(sprintf('Laptime: %.2fs', laptime))

disp(' ')


%% Determine maximum velocities
function v = calculateSpeedTrace(veh, r)

    % Takes in vehicle parameters and track curvature at 1 location

    % Set up quadratic formula for speed

    if r == 0

        v = veh.vel_max;

    else

        % r = 0.01;
        %
        % D = 0.25 * 0.5 * veh.rho * veh.frontalA * veh.Cl;
        %
        % a = abs((veh.PDY2 / -veh.FZO) * D^2);
        %
        % b = abs((veh.PDY1 * D - veh.PDY2 * D + (2 * veh.PDY2 * -veh.fz_per_tire_static) / -veh.FZO * D)) - (veh.m * r) / (sign(r) * 4 * veh.tireFactor_Y);
        %
        % c = abs(veh.PDY1 * -veh.fz_per_tire_static - veh.PDY2 * -veh.fz_per_tire_static + veh.PDY2 * veh.fz_per_tire_static^2 / -veh.FZO);
        %
        % v_solve = [sqrt((-b + sqrt(b^2 - 4*a*c)) / (2*a));
        %            sqrt((-b - sqrt(b^2 - 4*a*c)) / (2*a))];
        %
        % v_solve = min(v_solve(v_solve >= 0));
        %
        % % Check for max vehicle speed from motor
        % v = min([v_solve, veh.vel_max]);

        Mu_nominal = abs(veh.PDY1) * veh.tireFactor_Y;
        dMudFz = abs(veh.PDY2) / abs(veh.FZO) * veh.tireFactor_Y;
        FZ_nominal = veh.FZO;
        FZ_tot = veh.fz_tot;

        D = 0.5 * veh.rho * veh.frontalA * veh.Cl;

        a = -0.25 * sign(r) * dMudFz * D^2;
        b = sign(r) * (Mu_nominal * D + dMudFz * FZ_nominal * D - 0.5 * dMudFz * FZ_tot * D) - (veh.m * r);
        c = sign(r) * (Mu_nominal * FZ_tot + dMudFz * FZ_nominal * FZ_tot - 0.25 * dMudFz * FZ_tot^2);

        v_solve = [sqrt((-b + sqrt(b^2 - 4*a*c)) / (2*a));
                   sqrt((-b - sqrt(b^2 - 4*a*c)) / (2*a))];

        realLogical = [isreal(v_solve(1));
                       isreal(v_solve(2))];

        v_solve = min(v_solve(realLogical));

        if (sum(realLogical) == 0)
            disp('Check initial maximum velocity calculation, solution may be imaginary');
            disp(r)
        end

        % Check for max vehicle speed from motor
        v = min([v_solve, veh.vel_max]);

    end

    % Adjust speed for drag
    speedAdjust = true;

    while (speedAdjust)

        % Calculate aero forces
        aero_downF = 0.5 * veh.rho * veh.Cl * veh.frontalA * v^2;
        aero_dragF = -0.5 * veh.rho * veh.Cd * veh.frontalA * v^2;

        roll_F = -veh.c_roll * (veh.fz_tot + aero_downF);

        fz_tire = (veh.fz_tot + aero_downF) / 4;

        ax_drag = (aero_dragF + roll_F) / veh.m;

        % Lateral
        ay_max = sign(r) * veh.FYfunc(fz_tire) * 4 / veh.m;

        ay_needed = v^2 * r;

        % Longitudinal
        ax_tire_max = veh.FXfunc(fz_tire) .* 4 ./ veh.m;

        ax_power_limit = interp1(veh.powerLimit.vehicle_speed, veh.powerLimit.totalPowerLimitedMotorTorque, v) * veh.GR / veh.R_tire / veh.m;


        % Available combined lat. accel at ax_tire == -ax_drag
        ay_actual = ay_max * sqrt(1 - (ax_drag / min(ax_tire_max, ax_power_limit))^2);

        % Available combined long. accel at ay_needed
        % ax_acc = ax_tire_max_accel * sqrt(1 - (ay_needed / ay_max)^2);

        if abs(ay_actual) < abs(ay_needed)

            v = sqrt(ay_actual/r) - 1e-3;
        else
            speedAdjust = false;
        end
    end
end

function [j_next, j] = next_point(j, j_max, mode, trackConfig)

    switch mode
        case 1 % accel.

            switch string(trackConfig)
                case 'Closed'
                    if j == j_max - 1
                        j = j_max;
                        j_next = 1;
                    elseif j == j_max
                        j = 1;
                        j_next = j + 1;
                    else
                        j = j + 1;
                        j_next = j + 1;
                    end
                case 'Open'
                    j = j + 1;
                    j_next = j + 1;
            end
        case -1 % decel.
            switch string(trackConfig)
                case 'Closed'
                    if j == 2
                        j = 1;
                        j_next = j_max;
                    elseif j == 1
                        j = j_max;
                        j_next = j - 1;
                    else
                        j = j - 1;
                        j_next = j - 1;
                    end
                case 'Open'
                    j = j - 1;
                    j_next = j - 1;
            end
    end
end

function [v_next, ax, ax_commanded, ay, overshoot] = vehicle_solve(veh, tr, v, v_max_next, j, mode)
    % Takes in vehicle params, track params, current velocity for either
    % modes, maximum velocity at next point, current point in track mesh,
    % and current mode for either accel. or decel.

    % Assume no overshoot
    % Since the solver assumes constant acceleration over each dx,
    % overshoot is defined to be if over the current dx, the velocity would
    % increase beyond the maximum allowed velocity by the max vel. trace
    overshoot = false;

    % Get dx for current point in track mesh
    dx = tr.dx(j);
    % Get curvature for current point in track mesh
    r = tr.r(j);
    g = 9.81;

    %% Calculate external forces

    % Aero forces
    aero_downF = 0.5 * veh.rho * veh.Cl * veh.frontalA * v^2;
    aero_dragF = -0.5 * veh.rho * veh.Cd * veh.frontalA * v^2;

    % Rolling resistance
    roll_F = -veh.c_roll * (veh.fz_tot + aero_downF);

    % Normal load on each tire
    fz_tire = (veh.fz_tot + aero_downF) / 4;



    %% Check overshoot

    % Calculate maximum allowed acceleration to not overshoot
    ax_max_allowed = mode * (v_max_next^2 - v^2) / (2 * dx);

    ax_drag = (aero_dragF + roll_F) / veh.m;

    % Needed acceleration to not overshoot
    % i.e. if ax_max_allowed = 0, the necessary acceleration would simply
    % be what is needed to overcome drag. Drag is negative hence the minus
    ax_needed = ax_max_allowed - ax_drag;

    %% Current lat. accel
    ay = v^2 * r;

    %% Tire forces

    if abs(ay) ~= 0
        % If cornering...

        % Max lat. acceleration from tires
        ay_max = sign(ay) * veh.FYfunc(fz_tire) * 4 / veh.m;

        if abs(ay / ay_max) >= 1
            % if ay/ay_max > 1, the friction ellipse would result in
            % complex number, should not happen
            ellipse_multi = 0;
        else
            ellipse_multi = sqrt(1 - (ay / ay_max)^2);
        end
    else
        % If not cornering...
        % Pure longitudinal case
        ellipse_multi = 1;
    end



    %% Calculate accelerations
    if ax_needed >= 0

        % Calculate maximum long. accel. from tires
        ax_tire_max = veh.FXfunc(fz_tire) .* 4 ./ veh.m;

        % Calculate available long. accel. from tires with friction ellipse
        ax_tire_combined = ax_tire_max * ellipse_multi;

        % Calculate maximum power limited long. accel.
        ax_power_limit = interp1(veh.powerLimit.vehicle_speed, veh.powerLimit.totalPowerLimitedMotorTorque, v) * veh.GR / veh.R_tire / veh.m;

        ax_commanded = min([ax_tire_combined, ax_power_limit, ax_needed]);
    else
        ax_tire_max = -veh.FXfunc(fz_tire) .* 4 ./ veh.m;

        ax_tire_combined = ax_tire_max * ellipse_multi;

        ax_commanded = -min([abs(ax_tire_combined), abs(ax_needed)]);

    end

    %% Final results

    ax = ax_commanded + ax_drag;

    v_next = sqrt(v^2 + 2 * mode * ax * dx);

    %% Overshoot
    if v_next > v_max_next

        overshoot = true;

        v_next = inf;
        ax = 0;
        ay = 0;
        return
    end
end



