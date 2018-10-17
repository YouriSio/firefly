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
% This controller enforces a general path described in T.
% The position along the path is determined by checking the closest point
% to the path near the previous point

    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(T);
            
        case 2
            sys=mdlUpdate(t,x,u);
            
        case 3
            sys=mdlOutputs(t,x,u,T,ParameterController);
            
        case {1, 4, 9} % No discrete states, so 1,4 and 9 not used.
            sys = [];
            
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end


%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================

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


%=============================================================================
% mdlUpdate
% Returns the updated state for the S-function
%=============================================================================

function sys = mdlUpdate(t,x,u)  %#ok<INUSD>
    sys = [];
end

%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================

function sys = mdlOutputs(t,x,u,T,ParameterController)   %#ok<INUSL>
   
    k1 = ParameterController.k1; %Velocity gain
    k2 = ParameterController.k2; %Position D

    %% Gather input and state 
   
    X_World = u(1);
    Y_World = u(2);
    Z_World = u(3);
    Psi_World = u(4);
    Vx_World= u(5);
    Vy_World = u(6);
    Vz_World = u(7);
    Vpsi_World = u(8);
    t = u(9); %time

    % -------------------------------------------------------------------------
    %% PD law with or without feedforward
    
    indt = min(max(floor(t/T.period),1),size(T.X,2));
    Xdes = T.X(indt);
    Ydes = T.Y(indt);
    Zdes = T.Z(indt);
    Psides = T.PSI(indt);
    Vxdes = T.VX(indt);
    Vydes = T.VY(indt);
    Vzdes = T.VZ(indt);
    Vpsides = T.VPSI(indt);
    Axdes = T.AX(indt);
    Aydes = T.AY(indt);
    Azdes = T.AZ(indt);
    Apsides = T.APSI(indt);
    
    if ParameterController.feedforward == 1
        Ax    = k1*(Xdes - X_World) + k2*(Vxdes-Vx_World) + Axdes;
        Ay    = k1*(Ydes - Y_World) + k2*(Vydes-Vy_World) + Aydes;
        Az    = k1*(Zdes - Z_World) + k2*(Vzdes-Vz_World) + Azdes;
        Apsi  = k1*(Psides - Psi_World) + k2*(Vpsides-Vpsi_World) + Apsides;
    else
        Ax    = k1*(Xdes - X_World) + k2*(-Vx_World);
        Ay    = k1*(Ydes - Y_World) + k2*(-Vy_World);
        Az    = k1*(Zdes - Z_World) + k2*(-Vz_World);
        Apsi  = k1*(Psides - Psi_World) + k2*(-Vpsi_World);
    end

    %% Output
    sys = [Ax;Ay;Az;Apsi];
    % -------------------------------------------------------------------------
end 

