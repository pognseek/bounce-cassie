% Simulation for bouncing in 2D
clear all;

%% Parameters
% Fixed
mb = 1;             % Mass of the ball (kg)
mp = 10;            % Mass of the paddle (kg)
g  = 9.81;          % Gravity (m/s^2)
params.rb = 0.1;    % Radius of the ball (m)
params.P  = 0;      % Height of the paddle (m)
params.PL = 0.5;    % Length of the paddle (m)
th_des = pi/2;      % If pi/2, goal: bounce ball vertically in same spot

% Variable
H0  = 0.5 + rand;   % Initial height (m)
vb0 = rand;         % Initial vertical velocity (m/s)
ke  = rand;         % Coefficient of restitution
params.H  = abs(randn) + params.rb; % Desired steady-state apex height (m)
vdes = sqrt(2*g*(params.H - params.rb)); % Desired velocity for H

% 2D Variables
x0  = 0;            % Initial horizontal position
dx0 = randn;          % Initial horizontal velocity
xp0 = 0;            % Initial horizontal position of the paddle
th0 = 0;            % Initial angle of the paddle
dxp0 = 0; dzp0 = 0; dthp0 = 0;  % Paddle velocities (rest)
dthmax = 1000*pi;    % Maximum rotational velocity (100% made-up)

est = true;        % Should we estimate coefficient of restitution?

% Simulation parameters
T  = 3;     % Length of simulation (s)
dt = 1e-4;  % Step size (s)
printon = true;

%% Initialization
% States
x      = [x0; H0; dx0; vb0; ... % Ball states: x, z, dx, dz
    xp0; params.P; th0; dxp0; dzp0; dthp0]; % Paddle: x, z, th, dx, dz, dth
xi     = x;         % Initial states (for flight phase)
t      = 0;         % Time at start of simulation
t_imp  = 0;         % Time of xi
flight = true;      % Phase
nbounce= 0;         % Number of bounces

% Initialize matrices for storage
tmat = []; xmat = []; phase = []; vpmat = [];

% Estimate coefficient of restitution
if est
    kest = 0.51;
else
    kest = ke;
end

%% Simulation
while t < T
    xpre = x;
    % Ball in flight
    if flight
        % Ball dynamics, ballistic motion
        x(1) = xi(1) + xi(3) * (t - t_imp); % Horizontal position
        x(2) = xi(2) + xi(4) * (t - t_imp) - g/2 * (t - t_imp)^2;   % Height
        x(3) = xi(3);   % No horizontal acceleration
        x(4) = xi(4) - g * (t - t_imp); % Vertical velocity with gravity
        
        % Paddle dynamics, track ball
        % Positions of paddle
        x(5) = x(1);    % Horizontally track ball
        x(6) = x(6);    % Vertically stay put
        if abs(x(3)) > 1e-3
            th_p = (th_des + atan(x(4)/x(3))) / 2;    % Angle track
        else
            th_p = 0;    % Stay put if ball bouncing straight up
        end
        
        % Velocities
        x(8) = xpre(5) - x(5);  % Horizontally track ball
        x(9) = xpre(6) - x(6);  % Vertically stay put
%         x(10)= min(-dthmax, max((th_p - xpre(7))/dt, dthmax));
        x(10)= (th_p - xpre(7)) / dt;
        
        % Adjust angle
        x(7) = xpre(7) + dt * x(10);
        
        % Impact?
        if (x(2) <= (x(6) + params.rb)) && (x(4) < 0) % Hits paddle after falling
            flight = false; % Impact event
            t_imp  = t;     % Indicate time of impact
        end
    else    % Impact
        if printon; fprintf([num2str(kest-ke) ' bounce ' num2str(t) '\n']); end
        
        % Calculations: before impact
        th_ball_pre = atan(x(4) / x(3));        % Angle of the ball velocity
        v_ball_pre  = -sqrt(x(3)^2 + x(4)^2);   % Magnitude of ball velocity
%         th_p = (th_des + th_ball_pre) / 2;      % Desired angle of the paddle
        
        % Calculations: angle after impact
        th_ball_post = - th_ball_pre + 2*th_p;  % Angle of ball velocity, post-impact
        
        % Calculations: paddle states
        th_p = x(7);    % Angle calculated during flight
        vp = 1/(1+kest) * (vdes/sin(th_ball_post) + kest*v_ball_pre);
        
        % Calculations: velocity of ball after impact
        v_post = -ke * v_ball_pre + (1+ke) * vp; % Magnitude of ball velocity, post-impact
        
        % Ball dynamics: position
        x(1) = x(1);    % No deformation
        x(2) = x(2);    % No deformation
        x(3) = v_post * cos(th_ball_post);
        x(4) = v_post * sin(th_ball_post);
        
        % Paddle: no dynamics
%         x(5) = x(5);
%         x(6) = x(6);
        x(7) = th_p;    % New angle of the paddle
        
        x(8) = vp * sin(th_p);
        x(9) = vp * cos(th_p);
        x(10)= x(10);
        
        % Reaction of paddle
        x(5) = x(5) + x(8) * dt;
        x(6) = x(6) + x(9) * dt;
        
        vpmat = [vpmat; length(tmat)+1, th_p, x(8), x(9)];
        
        % Estimate
        if est
            vpostest = sqrt(x(3)^2 + x(4)^2);
            kest = (vpostest - vp) / (vp - v_ball_pre);
        end
        
        % Update
        xi = x; % States at beginning of flight
        flight = true;
        nbounce = nbounce + 1;
    end
    
    % Store and update
    xmat  = [xmat x];       % States
    tmat  = [tmat t];       % Time
    phase = [phase flight]; % Phase
    
    t = t + dt;
    
    if nbounce > 10
        break;
    end
end

%% Plot

figure(100);    % Ball states
a(1) = subplot(2,3,1); plot(tmat',xmat([1 5],:)','.-'); % Horizontal
title('Horizontal position')
a(2) = subplot(2,3,2); plot(tmat',xmat([2 6],:)','.-'); % Vertical
hold on; plot(tmat',phase'*params.rb);              % Phase
plot([tmat(1) tmat(end)],params.H*[1 1]); hold off  % Desired height
title(['H_{des} = ' num2str(params.H)]);
a(3) = subplot(2,3,3); plot(tmat',xmat(7,:)','.-'); % Angle
hold on; plot(tmat(vpmat(:,1))', vpmat(:,2),'ro'); hold off; % Paddle angle at impact
title('Angle')

a(4) = subplot(2,3,4); plot(tmat',xmat([3 8],:)','.-'); % Horizontal velocity
hold on; plot([tmat(1) tmat(end)],[0 0]);           % Want velocity = 0
plot(tmat(vpmat(:,1))', vpmat(:,3),'ro'); hold off; % Paddle velocity at impact
title(['dx_{des} = 0']);
a(5) = subplot(2,3,5); plot(tmat',xmat([4 9],:)','.-'); % Vertical velocity
hold on; plot(tmat(vpmat(:,1))', vpmat(:,4),'ro'); hold off; % Paddle velocity at impact
title('dz')
a(6) = subplot(2,3,6); plot(tmat',xmat(10,:)','.-');    % Angular velocity
title('d\theta')
linkaxes(a,'x');

animatefun(tmat',xmat',params);





