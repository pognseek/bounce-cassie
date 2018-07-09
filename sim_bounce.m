% Simulation for bouncing in 1D
clear all;

%% Parameters
% Fixed
mb = 1;             % Mass of the ball (kg)
mp = 10;            % Mass of the paddle (kg)
g  = 9.81;          % Gravity (m/s^2)
params.rb = 0.1;    % Radius of the ball (m)
params.P  = 0;      % Position of the paddle (m)
params.PL = 0.5;    % Length of the paddle (m)

% Variable
H0  = 0.5 + rand;   % Initial height (m)
vb0 = rand;         % Initial vertical velocity (m/s)
ke  = rand;         % Coefficient of restitution
params.H  = abs(randn) + params.rb; % Desired steady-state apex height (m)
vdes = sqrt(2*g*(params.H - params.rb)); % Desired velocity for H

est = true;        % Should we estimate coefficient of restitution?

% Simulation parameters
T  = 3;     % Length of simulation (s)
dt = 1e-4;  % Step size (s)
printon = true;

%% Initialization
% States
x      = [H0; vb0]; % Position, velocity of the ball
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
        x(1) = xi(1) + xi(2) * (t - t_imp) - g/2 * (t - t_imp)^2;
        x(2) = xi(2) - g * (t - t_imp);
        
        % Impact?
        if (x(1) <= (params.P + params.rb)) && (x(2) < 0) % Hits paddle after falling
            flight = false; % Impact event
            t_imp  = t;     % Indicate time of impact
        end
    else    % Impact
        if printon; fprintf([num2str(kest-ke) ' bounce ' num2str(t) '\n']); end
        
        % Ball dynamics: position
        x(1) = x(1);    % No deformation
        
        % Calculate desired paddle velocity
        vp = 1/(1+kest) * (kest * x(2) + vdes); vpmat = [vpmat; vp];
        vpre = x(2);    % Ball velocity pre-impact
        
        % Ball dynamics: velocity
        x(2) = -ke * x(2) + (1+ke) * vp;
        
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





