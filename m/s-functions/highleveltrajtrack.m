%% High level control of the quad
%  Sees the quad as a double integrator, 
%  receiving 3D Position + Yaw angle and Velocity + Yaw Rate and outputting acceleration.
%  Standard proportional derivative control law with the option for feedback 
%  or not set by flag ParameterController.feedforward.
%
% Input arguments u with
% X_world = u(1);
% Y_world = u(2);
% Z_world = u(3);
% Psi_world = u(4);
% Vx_world= u(5);
% Vy_world = u(6);
% Vz_world = u(7);
% Vpsi_world = u(8);
% t = u(9); %time
%
% Output arguments: four accelerations [ax;ay;az;apsi];

%% S-Function definition
function [sys,x0,str,ts,simStateCompliance] = highleveltrajtrack(t,x,u,flag, T,ParameterController)
    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(T);
        case 3
            sys=mdlOutputs(t,x,u,T,ParameterController);  
        case {1, 2, 4, 9} % No discrete states, so 1,4 and 9 not used.
            sys = [];  
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

% Return the sizes, initial conditions, and sample times for the S-function.
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(T)
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
    ts = [T.period 0];
    simStateCompliance = 'DefaultSimState';
end

% Return the output vector for the S-function
function sys = mdlOutputs(~,~,u,T,ParameterController)
    k1 = ParameterController.k1; %Velocity gain
    k2 = ParameterController.k2; %Position D

    % Gather input and state 
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
    indt = min(max(floor(t/T.period),1),size(T.X,2));
    
    x_des    = T.X(indt);
    y_des    = T.Y(indt);
    z_des    = T.Z(indt);
    psi_des  = T.PSI(indt);
    vx_des   = T.VX(indt);
    vy_des   = T.VY(indt);
    vz_des   = T.VZ(indt);
    vpsi_des = T.VPSI(indt);
    ax_des   = T.AX(indt);
    ay_des   = T.AY(indt);
    az_des   = T.AZ(indt);
    apsi_des = T.APSI(indt);
    
    if ParameterController.feedforward == 1
        ax    = k1*(x_des - x_world) + k2*(vx_des-vx_world) + ax_des;
        ay    = k1*(y_des - y_world) + k2*(vy_des-vy_world) + ay_des;
        az    = k1*(z_des - z_world) + k2*(vz_des-vz_world) + az_des;
        apsi  = k1*(psi_des - psi_world) + k2*(vpsi_des-vpsi_world) + apsi_des;
    else
        ax    = k1*(x_des - x_world) + k2*(-vx_world);
        ay    = k1*(y_des - y_world) + k2*(-vy_world);
        az    = k1*(z_des - z_world) + k2*(-vz_world);
        apsi  = k1*(psi_des - psi_world) + k2*(-vpsi_world);
    end

    % Output
    sys = [ax;ay;az;apsi];
end 