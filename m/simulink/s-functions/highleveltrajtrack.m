%% High level control of the quad
% This s-function computes the optimal acceleration in each of the four
% directions to get to a wanted state (read: position). It does this by
% comparing the actual state with the wanted state through some algorithm.

%% The S-function
function [sys,x0,str,ts,simStateCompliance] = highleveltrajtrack(t,x,u,flag, reftraj,ParameterController)
    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(reftraj);
        case 3
            sys=mdlOutputs(t,x,u,reftraj,ParameterController);  
        case {1, 2, 4, 9} % No discrete states, so 1,4 and 9 not used.
            sys = [];  
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

% Return the sizes, initial conditions, and sample times for the S-function.
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(reftraj)
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 4;  % dynamically sized
    sizes.NumInputs      = 9;  % dynamically sized
    sizes.DirFeedthrough = 1;  % has direct feedthrough
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    str = [];
    x0  = [];
    ts = [reftraj.period 0];
    simStateCompliance = 'DefaultSimState';
end

% Return the output vector for the S-function
function sys = mdlOutputs(~,~,u,reftraj,ParameterController)
    k1 = ParameterController.k1; %Velocity gain
    k2 = ParameterController.k2; %Position D

    % Gather current state
    x_world    = u(1);
    y_world    = u(2);
    z_world    = u(3);
    psi_world  = u(4);
    vx_world   = u(5);
    vy_world   = u(6);
    vz_world   = u(7);
    vpsi_world = u(8);
    t          = u(9);

    % PD law with or without feedforward
    indt = min(max(floor(t/reftraj.period),1),size(reftraj.x,2));
    
    % Gather wanted state
    x_des    = reftraj.x(indt);
    y_des    = reftraj.y(indt);
    z_des    = reftraj.z(indt);
    psi_des  = reftraj.psi(indt);
    vx_des   = reftraj.vx(indt);
    vy_des   = reftraj.vy(indt);
    vz_des   = reftraj.vz(indt);
    vpsi_des = reftraj.vpsi(indt);
    ax_des   = reftraj.ax(indt);
    ay_des   = reftraj.ay(indt);
    az_des   = reftraj.az(indt);
    apsi_des = reftraj.apsi(indt);
    
    % Compute acceleration to reach wanted state
    if ParameterController.feedforward == 1
        ax   = k1*(x_des - x_world) + k2*(vx_des-vx_world) + ax_des;
        ay   = k1*(y_des - y_world) + k2*(vy_des-vy_world) + ay_des;
        az   = k1*(z_des - z_world) + k2*(vz_des-vz_world) + az_des;
        apsi = k1*(psi_des - psi_world) + k2*(vpsi_des-vpsi_world) + apsi_des;
    else
        ax   = k1*(x_des - x_world) + k2*(-vx_world);
        ay   = k1*(y_des - y_world) + k2*(-vy_world);
        az   = k1*(z_des - z_world) + k2*(-vz_world);
        apsi = k1*(psi_des - psi_world) + k2*(-vpsi_world);
    end

    % Output
    sys = [ax; ay; az; apsi];
end 