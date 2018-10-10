%% High level control of the quad
%  Sees the quad as a double integrator, 
%  receiving 3D position+yaw angle and velocity+yaw rate and outputting acceleration.
%  Standard proportional derivative control law with the option for feedback 
%  or not set by flag parcontroller.feedforward.
%
% Input arguments u with
% x_world = u(1);
% y_world = u(2);
% z_world = u(3);
% psi_world = u(4);
% vx_world= u(5);
% vy_world = u(6);
% vz_world = u(7);
% vpsi_world = u(8);
% t = u(9); %time
%
% Output arguments: four accelerations [ax;ay;az;apsi];

%% S-Function definition
function [sys,x0,str,ts,simStateCompliance] = highleveltrajtrack(t,x,u,flag, T,parcontroller)
% This controller enforces a general path described in T
% the position along the path is determined by checking the closest point
% to the path near the previous point

    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(T);
        case 2
            sys=mdlUpdate(t,x,u);
        case 3
            sys=mdlOutputs(t,x,u,T,parcontroller);
        case {1, 4, 9} % No discrete states, so 1,2 and 4 not used
            sys = [];
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(T)
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 4;  % dynamically sized
    sizes.NumInputs      = 9;  % dynamically sized
    sizes.DirFeedthrough = 1;   % has direct feedthrough
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    str = [];
    x0  = [];
    ts = [T.period 0];
    simStateCompliance = 'DefaultSimState';
end

%
%=============================================================================
% mdlUpdate
% Returns the updated state for the S-function
%=============================================================================
%
function sys = mdlUpdate(t,x,u)
    sys = [];
end
%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u,T,parcontroller)
    % magic numbers
    k1 = parcontroller.k1; %Velocity gain
    k2 = parcontroller.k2; %Position D
    k3 = parcontroller.k3; %Position P

    %% gather input, and state 
    %state
    x_world = u(1);
    y_world = u(2);
    z_world = u(3);
    psi_world = u(4);
    vx_world= u(5);
    vy_world = u(6);
    vz_world = u(7);
    vpsi_world = u(8);
    t = u(9); %time

    % -------------------------------------------------------------------------
    %% PD law with or without feedforward
    indt = min(max(floor(t/T.period),1),size(T.X,2));
    xdes = T.X(indt);
    ydes = T.Y(indt);
    zdes = T.Z(indt);
    psides = T.PSI(indt);
    vxdes = T.VX(indt);
    vydes = T.VY(indt);
    vzdes = T.VZ(indt);
    vpsides = T.VPSI(indt);
    axdes = T.AX(indt);
    aydes = T.AY(indt);
    azdes = T.AZ(indt);
    apsides = T.APSI(indt);
    
    if parcontroller.feedforward == 1
        ax = k1*(xdes - x_world) + k2*(vxdes-vx_world) + axdes;
        ay = k1*(ydes - y_world) + k2*(vydes-vy_world) + aydes;
        az = k1*(zdes - z_world) + k2*(vzdes-vz_world) + azdes;
        apsi = k1*(psides - psi_world) + k2*(vpsides-vpsi_world) + apsides;
    else
        ax = k1*(xdes - x_world) + k2*(-vx_world);
        ay = k1*(ydes - y_world) + k2*(-vy_world);
        az = k1*(zdes - z_world) + k2*(-vz_world);
        apsi = k1*(psides - psi_world) + k2*(-vpsi_world);
    end

    %% Output
    sys = [ax;ay;az;apsi];
end 

