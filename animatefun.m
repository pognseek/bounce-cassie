function animatefun(t,x,params)
% Animate the ball

figure(1000);
fignum = gcf;

wball = transpose(-params.rb:0.01:params.rb);
tball = sqrt(params.rb^2 - wball.^2);
bball = -sqrt(params.rb^2 - wball.^2);

dt = 0.01;
tspan = [t(1):dt:t(end)]';
if size(x,2) > 2
    coords = [x(:,1) x(:,3)];
else
    coords = [x(:,1) params.P*ones(size(x(:,1)))];
end
intcoords = interp1(t,coords,tspan,'spline');

ax = ceil(max(params.rb,params.PL/2));
aymax = max(x(:,1) + params.rb*2);
aymin = params.P - params.rb*2;
ay = (aymax + aymin) / 2;
% a = max(ax,ay);
a = max(ax,(aymax - aymin)/2);

for i=1:length(tspan)
    figure(fignum);
    
    % Ball
    plot(wball,tball+intcoords(i,1),'b')
    hold on
    plot(wball,bball+intcoords(i,1),'b')
    plot(0,intcoords(i,1),'rx')
    
    % Paddle
    plot([-params.PL/2 params.PL/2],[intcoords(i,2) intcoords(i,2)],'k')
%     hold off
    
    % Goal
%     plot([-a/2 a/2],[params.H params.H],'r--')
    plot([-a/2 a/2],[params.H params.H],'r--')
    hold off
    
    
    title(['t = ' num2str(tspan(i))]);
    axis([-a a ay-a ay+a]); axis square;
    
    pause(dt/100);
end


end