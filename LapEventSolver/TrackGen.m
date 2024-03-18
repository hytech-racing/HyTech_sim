function [track] = TrackGen(filename, kappa, dispGraph)
    
    mesh_size = 1; % m
    
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
    
            % Split up the current long angle segment into l_inj + (l-2*l_inj)
            % + l_inj (first, middle, and last piece)
    
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
    angle_seg = lambda * rad2deg(dx.*-r);
    
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
    
    if(dispGraph)
    
        warning off 

        figure
        tiledlayout(1, 2)
        nexttile
        hold on
        box on
        grid on
        axis equal
        daspect([1 1 1])
        plot(X, Y)
        plot(X(1), Y(1), 'o')
        line([X(1) X(15)], [Y(1) Y(15)], 'Color','red','LineWidth',1)
        
        for i = 1:length(w)
            plot(X(round(apexLocation(i)+w(i)/2)), Y(round(apexLocation(i)+w(i)/2)), 'm*')
        end
        legend('Track', 'Starting Point', 'Travel Direction', 'Apex Approx. Center')
        
        title('Track Map')
        
        nexttile
        hold on
        box on
        grid on
        plot(x, r)
        title('Curvature')
        xlabel('Position [m]')
        ylabel('Curvature [m^-^1]')
    
        warning on
    end
    
    track.X = X;
    track.Y = Y;
    track.dx = dx;
    track.r = r;
    track.r_apex = r_apex;
    track.n = n;
    track.posAlongTrack = cumsum(dx);
    track.config = config_str;

