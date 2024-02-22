% tiledlayout(2,1)
h = animatedline;
q = animatedline;
g = animatedline;

% figure
j = 1;

for idx = 1:0.01:100
    subplot(2,1,1)
    x(j) = sin(idx);
    plot(x, '.')
    title('idx')
    j=j+1;
    drawnow limitrate
    
    % subplot(3,1,1)
    % addpoints(h,idx,idx)
    % drawnow
    % 
    % subplot(3,1,2)
    % addpoints(q,idx,idx/2)
    % drawnow
    % 
    % 
    % subplot(3,1,3)
    % addpoints(g,idx,idx/3)
    % drawnow



end
