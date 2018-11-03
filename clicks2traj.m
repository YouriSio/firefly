%% Define the trajectory by clicking (x,y) coordinates
%
% Using Matlab built in function getpts.m it starts by asking the user to 
% introduce some points on the image, which for convenience if the image of
% the soccer field: these points can directly be used by the simulation with
% the soccer field environment. Click once for introducing points and twice 
% for the final point. This defines the x, and y coordinates. The z and psi 
% coordinates as well as the times at which the drone should reach each way 
% point are defined automatically or can be customized by the user. Many 
% control options allow the user to define this as well as using previously 
% clicked points. After this, it runs the initscript.

%% Clean up
clear;
close all;
clc;

%% Options
option.constantspeed  = false;  % Defines if the drone will follow a constant speed
option.constantheight = false;  % Defines if the drone will follow a constant height
option.constantyaw    = true;  % Defines if the drone will have a constant yaw
option.loadwaypoints  = true; % Defines if the trajectory will be loaded from waypoints.mat
option.savewaypoints  = false;  % Defines if the trajectory will be saved to waypoints.mat
option.saveT          = true;  % Defines if the trajectory will be saved to T.mat
option.plot           = true;  % Defines if the resulting trajectory will be plotted

%% Constants
speed = 1;                     % Will be used if constantspeed is true
height = 1;                    % WIll be used if constantheight is true
field_center = [346 281];      % Center of the soccerfield image in pixels
field_dim = [8 10];            % Length of the soccerfield in meters
rho = 0.001;                   % Weighting factor for control input
m = 4;                         % Mass of the drone, needs to be larger than 2
tau = 0.1;                     % Input sampling period
N12 = 10;                      % Ratio of input sampling period over output sampling period

%% Obtain the way points
if option.loadwaypoints
    load('./mat/waypoints');
else
    imshow './_Aux/soccerfieldpic.jpg'
    [xi,yi] = getpts;
    p = [ -field_dim(1)/2*(yi-field_center(2))/field_center(2) field_dim(2)/2*(xi-field_center(1))/field_center(2)]; 
    px = p(:,1);
    py = p(:,2);
    
    if option.savewaypoints
        save('./mat/waypoints', 'px', 'py');
    end
end

if option.constantheight
    pz = -height*ones(size(px)); % Negative due to convention
else
    pz = -ones(size(px));
    pz(1) = -50; pz(2) = 0; % Customize here
end

N = length(px);
Tp = zeros(1,N);
Tp(1) = 0;
P = [px'; py'; pz'];

if option.constantspeed
    epsilon = 0.01;
    for i = 2:N
        % If two points are two close, spend epsilon seconds
        Tp(i) = Tp(i-1)+max(norm(P(:,i)-P(:,i-1))/speed, epsilon);
    end
else
    Tp = time'; % transpose waypoint data to a row
end

if option.constantyaw
    psi = zeros(1,N);
else
    psi = psi'; % transpose waypoint data to a row
end

%% Generate trajectory from the way points
X0 = [P(:,1) speed*(P(:,2)-P(:,1))/Tp(2) zeros(3,m-2)];
[xix,xiy,xiz]  = lqrtrajgeneration(P,Tp,X0,rho,tau,N12);

Ppsi = [psi; zeros(size(psi)); zeros(size(psi))];
PSI0 = [Ppsi(:,1) (Ppsi(:,2)-Ppsi(:,1))/Tp(2) zeros(3,m-2)];
[xipsi,~,~] = lqrtrajgeneration(Ppsi,Tp,PSI0,rho,tau,N12);

%% Plot
if option.plot
    plot3(P(1,:), P(2,:), P(3,:));
    hold on;
    plot3(xix(1,:), xiy(1,:), xiz(1,:));
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    set(gca, 'ZDir','reverse');
    set(gca, 'YDir','reverse');
    view(-70,40);
    Xsf = [field_dim(1), field_dim(1), -field_dim(1), -field_dim(1)]/2;
    Ysf = [field_dim(2), -field_dim(2), -field_dim(2),  field_dim(2)]/2;
    patch(Xsf,Ysf,[0 0 0 0],[0.1 1 0.1]);
end

delta = 0.1;
axis( [-field_dim(1)/2+delta field_dim(1)/2+delta -field_dim(2)/2+delta field_dim(2)/2+delta -3 +delta])

%% T format for simulation and run initscript  
T.X    = xix(1,:);             % X  reference
T.Y    = xiy(1,:);             % Y  reference
T.Z    = xiz(1,:);             % Z  reference
T.PSI  = xipsi(1,:);           % Yaw reference
T.VX   = xix(2,:);             % X velocity reference
T.VY   = xiy(2,:);             % Y velocity reference
T.VZ   = xiz(2,:);             % Z velocity reference
T.VPSI = xipsi(2,:);           % Yaw velocity reference
T.AX   = xix(3,:);             % X acceleration reference
T.AY   = xiy(3,:);             % Y acceleration reference
T.AZ   = xiz(3,:);             % Z acceleration reference
T.APSI = xipsi(3,:);           % Yaw acceleration reference
T.time = (1:size(T.X,2))*tau;  % Discrete-time
T.period = tau;                % Sampling period

% Save inside T.mat
if option.saveT
    assignin('base', 'T', T);
    save('./mat/T', 'T');
end
%Run initscript
initscript;