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
dx0 = 5;          % Initial horizontal velocity
xp0 = 0;            % Initial horizontal position of the paddle
th0 = 0;            % Initial angle of the paddle

est = false;        % Should we estimate coefficient of restitution?

% Simulation parameters
T  = 3;     % Length of simulation (s)
dt = 1e-4;  % Step size (s)
printon = true;

%% Initialization
% States
x      = [x0; H0; dx0; vb0; xp0; params.P; th0];   % xb, zb, dxb, dzb, xp, zp, thetap
xi     = x;         % Initial states (for flight phase)
t      = 0;         % Time at start of simulation
t_imp  = 0;         % Time of xi
flight = true;      % Phase

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
    % Ball in flight
    if flight
        % Ball dynamics, ballistic motion
        x(1) = xi(1) + xi(3) * (t - t_imp); % Horizontal position
        x(2) = xi(2) + xi(4) * (t - t_imp) - g/2 * (t - t_imp)^2;   % Height
        x(3) = xi(3);   % No horizontal acceleration
        x(4) = xi(4) - g * (t - t_imp); % Vertical velocity with gravity
        
        % Paddle dynamics, track ball
        x(5) = x(1);    % Horizontally track ball
        x(6) = x(6);    % Vertically stay put
        x(7) = x(7);    % Angle stay put
        
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
        th_p = (th_des + th_ball_pre) / 2;      % Desired angle of the paddle
        
        % Calculations: after impact
        th_ball_post = - th_ball_pre + 2*th_p;  % Angle of ball velocity, post-impact
        v_post = -ke * v_ball_pre + (1+ke) * 0; % Magnitude of ball velocity, post-impact
        
        % Ball dynamics: position
        x(1) = x(1);    % No deformation
        x(2) = x(2);    % No deformation
        x(3) = v_post * cos(th_ball_post);
        x(4) = v_post * sin(th_ball_post);
        
        % Paddle: no dynamics
        x(5) = x(5);
        x(6) = x(6);
        x(7) = th_p;    % New angle of the paddle
        
        % Estimate
        if est
            kest = (x(2) - vp) / (vp - vpre);
        end
        
        % Update
        xi = x; % States at beginning of flight
        flight = true;
    end
    
    % Store and update
    xmat  = [xmat x];       % States
    tmat  = [tmat t];       % Time
    phase = [phase flight]; % Phase
    
    t = t + dt;
end

%% Plot

figure(100);
a(1) = subplot(2,1,1); plot(tmat',xmat(1,:)','.-'); % Position
hold on; plot(tmat',phase'*params.rb);              % Phase
plot([tmat(1) tmat(end)],params.H*[1 1]); hold off  % Desired height
title(['H_{des} = ' num2str(params.H)]);
a(2) = subplot(2,1,2); plot(tmat',xmat(2,:)','.');  % Velocity
linkaxes(a,'x');

animatefun(tmat',xmat',params);





