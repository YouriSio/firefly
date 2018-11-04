%% Function points2traj
% Converts a set of points in 5D to an optimal trajectory along the points.
% Note that this reference trajectory is purely based on math and not
% physics - the simulation takes care of the physics part.

function [reftraj] = points2reftraj(coords)
    % Retrieve model parameters
    parcontroller = evalin('base', 'parcontroller');
    tau = parcontroller.tau;
    N12 = parcontroller.N12;
    rho = parcontroller.rho;
    
    % Extract coordinates
    t    = coords(:,1)'; % Discrete-time
    px   = coords(:,2)'; % X reference
    py   = coords(:,3)'; % Y reference
    psi  = coords(:,5)'; % Yaw reference
    
    if parcontroller.constantheight
        pz = -parcontroller.height*ones(size(px'))';
    else
        pz = coords(:,4)';
    end
    
    P = [px; py; pz];
    
    % Generate trajectory positions and derivatives
    X0 = [P(:,1) zeros(3,2)]; % Initial v = a = 0
    
    [xix,xiy,xiz]  = lqrtrajgeneration(P,t,X0,rho,tau,N12);
    
    % Generate trajectory yaw and derivatives
    Ppsi = [psi; zeros(size(psi)); zeros(size(psi))];
    PSI0 = [Ppsi(:,1) zeros(3,2)];
    [xipsi,~,~] = lqrtrajgeneration(Ppsi,t,PSI0,rho,tau,N12);
    
    % T format for simulation and run initscript  
    reftraj.x    = xix(1,:);                       % X reference
    reftraj.y    = xiy(1,:);                       % Y reference
    reftraj.z    = xiz(1,:);                       % Z reference
    reftraj.psi  = xipsi(1,:);                     % Yaw reference
    reftraj.vx   = xix(2,:);                       % X velocity reference
    reftraj.vy   = xiy(2,:);                       % Y velocity reference
    reftraj.vz   = xiz(2,:);                       % Z velocity reference
    reftraj.vpsi = xipsi(2,:);                     % Yaw velocity reference
    reftraj.ax   = xix(3,:);                       % X acceleration reference
    reftraj.ay   = xiy(3,:);                       % Y acceleration reference
    reftraj.az   = xiz(3,:);                       % Z acceleration reference
    reftraj.apsi = xipsi(3,:);                     % Yaw acceleration reference
    reftraj.t    = (1:size(reftraj.x,2))*tau/N12;  % Discrete-time
    reftraj.period = tau/N12;                      % Sampling period
end