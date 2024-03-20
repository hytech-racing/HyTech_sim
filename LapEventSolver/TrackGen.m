function [track] = TrackGen(filename, kappa, dispGraph)
    
    mesh_size = 0.1; % m
    
    % kappa = long corner adjuster

    lambda = 1;
    
    warning off
    track_shape = readtable(filename);
    warning on
    
    % Segment radius
    R = table2array(track_shape(:, 3));
    
    % Correct straights to infinite radius
    R(R == 0) = inf;
    
    % Segment length
    l = table2array(track_shape(:, 2));
    
    % Total length
    L = sum(l);
    
    config_str = table2array(track_shape(1, 4));
    
    config = zeros(1, 1);
    
    config(string(config_str) == 'Closed') = 1;
    config(string(config_str) == 'Open') = 0;
    
    % Pull strings for segment type
    type_str = table2array(track_shape(:, 1));
    
    type = zeros(length(l),1) ;
    
    type(string(type_str) == 'Straight') = 0 ;
    type(string(type_str) == 'Left') = -1 ;
    type(string(type_str) == 'Right') = 1 ;
    
    R(l == 0) = [];
    type(l == 0) = [];
    l(l == 0) = [];
    
    % Segment angle
    angle_seg = rad2deg(l./R);
    
    j = 1;
    RR = R;
    ll = l;
    tt = type;
    
    for i = 1:length(l)
    
        if angle_seg(i) > kappa
    
            % Calculate the length of the injected segment
            l_inj = min([ll(j) / 3, deg2rad(kappa) * R(i)]);
    
            % Split up the current long angle segment into 
            % l_inj + (l-2*l_inj) + l_inj (first, middle, and last piece)
    
            ll = [  ll(1:j-1); % Keep everything before the current segment
                    l_inj; % Add first piece
                    ll(j) - 2*l_inj; % Add middle piece
                    l_inj; % Add last piece
                    ll(j+1:end) % Keep everything after the current segment
                    ];
    
            RR = [  RR(1:j-1);
                    RR(j); % Injected segments all have the same radius
                    RR(j);
                    RR(j);
                    RR(j+1:end)
                ];
    
            tt = [  tt(1:j-1);
                    tt(j); % Injected segments all have same turning type
                    tt(j);
                    tt(j);
                    tt(j+1:end)];
    
            j = j + 3; % Move to next segment
        else
            j  = j + 1; % Move to next segment
        end
    
        
    end
    
    R = RR;
    l = ll;
    type = tt;
    
    X = cumsum(l); % End position of each segment
    XC = X - l/2; % Mid point of each segment
    
    j = 1;
    x = zeros(length(X) + sum(R == inf), 1);
    r = zeros(length(X) + sum(R == inf), 1);
    
    for i = 1:length(X)
        if R(i) == inf
            % Inject the end and beginning of the straight
            x(j) = X(i) - l(i);
            x(j + 1) = X(i);
            j = j + 2;
        else
            % Save the center of the corner
            x(j) = XC(i);
            r(j) = type(i)./R(i); % Curvature
            j = j + 1;
        end
    end
    
    xx = x;
    
    % Mesh Track
    
    if(floor(L) < L)
    
        x = [(0:mesh_size:floor(L))'; L];
    
    else
    
        x = (0:mesh_size:floor(L))';
    
    end

    
    dx = diff(x);
    dx = [dx; dx(end)];
    
    n = length(x); % Number of points
    
    % New Curvataure
    r = interp1(xx, r, x, 'pchip', 'extrap');
    
    % Coordinates
    X = zeros(n, 1);
    Y = zeros(n, 1);
    
    % Segment angle
    angle_seg = lambda * rad2deg(dx.*r);
    
    % Heading angle 
    angle_head = cumsum(angle_seg);
    
    % Tangency Fix
    if (config == 1)
        dh = [...
                mod(angle_head(end),sign(angle_head(end))*360);...
                angle_head(end)-sign(angle_head(end))*360....
                ] ;
            [~,idx] = min(abs(dh)) ;
            dh = dh(idx) ;
            angle_head = angle_head-x/L*dh ;
            angle_seg = [angle_head(1);diff(angle_head)] ;
    end
    
    angle_head = angle_head - angle_head(1);
    
    for i = 2:n
        p = [X(i-1);
             Y(i-1);
             0];
    
        xyz = rotz(angle_head(i-1)) * [dx(i-1); 0; 0] + p;
    
        X(i) = xyz(1);
        Y(i) = xyz(2);
    end
    
    [~, apexLocation, w] = findpeaks(abs(r));
    
    r_apex = r(apexLocation);
    
    if (config == 1)
        DX = x/L*(X(1)-X(end)) ;
        DY = x/L*(Y(1)-Y(end)) ;
        % adding correction
        X = X+DX ;
        Y = Y+DY ;
    end
   

    %% Tangential Angle Calculation

    tangentialAngleRad = zeros(size(X));

    for i = 1:length(X) - 1
        
        dXpos = X(i + 1) - X(i);
        dYpos = Y(i + 1) - Y(i);
        prevAngle = tangentialAngleRad(i);

        % atan2 only outputs [-pi, pi] or [-180, 180], to expand to [-2pi,
        % 2pi], mod is used.

        % Mod only outputs positive value, therefore angles such as -45
        % degrees becomes 315 degrees. To correct this, mod(-45, 360) needs
        % to be subtracted by 360 degrees (2pi). This corresponds to
        % whenever atan2(dYpos, dXpos) < 0, which is the same condition
        % for when mod(atan2(dYpos, dXpos)) > pi. 

        % However, this is also a problem for whenver the vehicle should 
        % finish a closed track because the final heading angle should be
        % 360 degrees. By subtracting 2 pi from whenever atan2(dYpos,
        % dXpos) < 0, any angle above magnitude of 180 degrees is
        % unachievable. For example, instead of going from 180 --> 360, the
        % curve would go from 180 --> -180 --> 0. The loop below checks the
        % "derivative" of the tangential angle to determine whether 2pi
        % should be subtracted. Essentially constraining the curve to be
        % continuous.

        potentialAngle = mod(atan2(dYpos, dXpos), 2*pi);

        % Any angular change above 1 rad threshold should subtract 2 pi
        if (abs(potentialAngle - prevAngle) > 1)
            potentialAngle = potentialAngle - 2*pi;
            tangentialAngleRad(i+1) = potentialAngle;
        else
            tangentialAngleRad(i+1) = potentialAngle;
        end

    end

    for i = 1:length(tangentialAngleRad) - 1

        dTangentialAng = tangentialAngleRad(i + 1) - tangentialAngleRad(i);

        if (dTangentialAng) < -1
            tangentialAngleRad(i + 1) = tangentialAngleRad(i);
        end

        if (dTangentialAng) > 1
            tangentialAngleRad(i + 1) = tangentialAngleRad(i);
        end

    end

%% Graphing

    if(dispGraph)
    
        warning off 

        figure
        tiledlayout(3, 1)
        nexttile
        hold on
        box on
        grid on
        axis equal
        daspect([1 1 1])
        plot(Y, X)
        plot(Y(1), X(1), 'o')
        line([Y(1) Y(15)], [X(1) X(15)], 'Color','red','LineWidth',1)
        
        for i = 1:length(w)
            plot(Y(round(apexLocation(i)+w(i)/2)), X(round(apexLocation(i)+w(i)/2)), 'm*')
        end
        legend('Track', 'Starting Point', 'Travel Direction', 'Apex Approx. Center')
        
        xlabel('Y [m]')
        ylabel('X [m]')
        title('Track Map')
        
        nexttile
        hold on
        box on
        grid on
        plot(cumsum(dx), r)
        title('Curvature')
        xlabel('Position Along Track [m] [m]')
        ylabel('Curvature [m^-^1]')
    
        nexttile
        hold on
        box on
        grid on
        plot(cumsum(dx), rad2deg(tangentialAngleRad))
        
        title('Tangential Angle')
        xlabel('Position Along Track [m]')
        ylabel('Tangential Angle [Deg]')

        warning on
    end
 
    

%% Output Track Data

    track.X = X;
    track.Y = Y;
    track.dx = dx;
    track.r = r;
    track.r_apex = r_apex;
    track.n = n;
    track.posAlongTrack = cumsum(dx);
    track.config = config_str;
    track.tangentialAngDeg = rad2deg(tangentialAngleRad);

end