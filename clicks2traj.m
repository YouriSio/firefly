%% Define the trajectory by clicking (x,y) coordinates
%
% Using Matlab built in function getpts.m it starts by asking the user to 
% introduce some points on the image, which for convenience if the image of
% the soccer field: these points can directly be used by the simulation with
% the soccer field environment. Click once for introducing points and twice 
% for the final point. This defines the x, and y coordinates. The z and psi 
% coordinates as well as the times at which the drone should reach each way 
% point are defined automatically or can be customized by the user. Many control 
% options allow the user to define this as well as using previously clicked points.
% After this, it runs the initscript.

%% Clean up
clear 
close all
clc

%% Options
option.constantspeed = true;   % Defines if the drone will follow a constant speed
option.constantheight = true;  % Defines if the drone will follow a constant height
option.constantyaw = true;     % Defines if the drone will have a constan yaw
option.loadwaypoints = false;  % Defines if the trajectory will be loaded from waypoints.mat
option.savewaypoints = true;   % Defines if the trajectory will be saved to waypoints.mat
option.saveT = true;           % Defines if the trajectory will be saved to T.mat
option.plot = true;            % Defines if the resulting trajectory will be plotted

%% Constants
speed = 1;                          % Will be used if constantspeed is true
height = 1;                         % WIll be used if constantheight is true
centersoccerfieldpic = [346 281];   % Center of the soccerfield image in pixels
lengthsoccerfield = [8 10];         % Length of the soccerfield in meters
rho = 0.001;                        % Weighting factor for control input
m = 4;                              % Mass of the drone, needs to be larger than 2
tau = 0.1;                          % Input sampling period
N12 = 10;                           % Ratio of input sampling period over output sampling period

%% Obtain the way points
if option.loadwaypoints == true
    load waypoints
else
    imshow './_Aux/soccerfieldpic.jpg'
    [xi,yi] = getpts;
    p = [ -lengthsoccerfield(1)/2*(yi-centersoccerfieldpic(2))/centersoccerfieldpic(2) lengthsoccerfield(2)/2*(xi-centersoccerfieldpic(1))/centersoccerfieldpic(2)]; 
    px = p(:,1);
    py = p(:,2);
    
    if option.savewaypoints == true
        save waypoints px py
    end
end

if option.constantheight == true
    pz = -height*ones(size(px)); % -1 due to convention
else
    pz = -ones(size(px));
    pz(1) = -2; pz(2) = -3; % customize here
end

N = length(px);
Tp = zeros(1,N);
Tp(1) = 0;
P = [px';py';pz'];

if option.constantspeed == true
    epsilon = 0.01;
    for i = 2:N
        % if two points are two close, spend epsilon sec
        Tp(i) = Tp(i-1)+max(norm(P(:,i)-P(:,i-1))/speed,epsilon);
    end
else
    % custom T
    Tp = 0:N-1; % customize here
end

if option.constantyaw == true
    psi = zeros(1,N);
else
    psi = zeros(1,N);
    psi(3) = pi/4; % customize yaw
    psi(4) = -pi/4;
    psi(5) = pi/4;
end

%% Generate trajectory from the way points
X0 = [P(:,1) speed*(P(:,2)-P(:,1))/Tp(2) zeros(3,m-2)];
[xix,xiy,xiz]  = lqrtrajgeneration(P,Tp,X0,rho,tau,N12);

Ppsi = [psi;zeros(size(psi));zeros(size(psi))];
PSI0 = [Ppsi(:,1) (Ppsi(:,2)-Ppsi(:,1))/Tp(2) zeros(3,m-2)];
[xipsi,~,~]  = lqrtrajgeneration(Ppsi,Tp,PSI0,rho,tau,N12);

%% Plot
if option.plot == true
    plot3(P(1,:),P(2,:),P(3,:))
    hold on
    plot3(xix(1,:),xiy(1,:),xiz(1,:))
    axis equal
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    set(gca, 'ZDir','reverse')
    set(gca, 'YDir','reverse')
    view(-70,40)
    Xsf = [lengthsoccerfield(1)/2 lengthsoccerfield(1)/2 -lengthsoccerfield(1)/2 -lengthsoccerfield(1)/2];
    Ysf = [lengthsoccerfield(2)/2 -lengthsoccerfield(2)/2 -lengthsoccerfield(2)/2  lengthsoccerfield(2)/2];
    patch(Xsf,Ysf,[0 0 0 0],[0.1 1 0.1])
end
delta = 0.1;
axis( [-lengthsoccerfield(1)/2+delta lengthsoccerfield(1)/2+delta -lengthsoccerfield(2)/2+delta lengthsoccerfield(2)/2+delta -3 +delta])
%% T format for simulation and run initscript  
T.X    = xix(1,:);             % X  reference
T.Y    = xiy(1,:);             % Y  reference
T.Z    = xiz(1,:);             % Z  reference
T.PSI  = xipsi(1,:);           % yaw reference
T.VX   = xix(2,:);             % x velocity reference
T.VY   = xiy(2,:);             % y velocity reference
T.VZ   = xiz(2,:);             % z velocity reference
T.VPSI = xipsi(2,:);           % yaw velocity reference
T.AX   = xix(3,:);             % x acceleration reference
T.AY   = xiy(3,:);             % y acceleration reference
T.AZ   = xiz(3,:);             % z acceleration reference
T.APSI = xipsi(3,:);           % yaw acceleration reference
T.time = (1:size(T.X,2))*tau;  % discrete-time
T.period = tau;                % sampling period

if option.saveT == true
    save T T
end

initscript