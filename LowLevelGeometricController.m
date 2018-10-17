%% Low level control of the quad, using a geometrically inspired control technique.
% S-Function to control the thurst vector and the psi angle
% It requires as input the desired accelerations coming from an outer loop controller, 
% the rotation matrix representing the attitute of the quad and the angular velocity. 
% It tries to track these desired accelerations by controlling the quad's thurst vector.
%
% Input arguments:
% u such that
% aref       = u(1:3);
% Psiref     = u(4);
% rx         = u(5:7);
% ry         = u(8:10);
% rz         = u(11:13);
% w           = u(14:16);
% where R = [rx ry rz] is the rotation matrix
%
% Output arguments
% control input u = [T \tau_\phi \tau_\theta \tau_\psi]

%% S-Function definition
function [sys,x0,str,ts,simStateCompliance] = LowLevelGeometricController(t,x,u,flag,parmodel,parcontroller)

switch flag

     case 0 % Initialize the states, sample times and state ordering strings.
     [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

     case 3 % Return the outputs of the S-function block.
     sys=mdlOutputs(t,x,u,parmodel,parcontroller); 

  case { 1, 2, 4, 9 }
    sys=[];

    otherwise % Return an error message for unhandled flag values
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()
% Return the sizes, initial conditions, and sample times for the S-function
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;  
sizes.NumInputs      = 16;  
sizes.DirFeedthrough = 1;   
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   % Inherited sample time

simStateCompliance = 'DefaultSimState'; %Simstate is same as the default one

function sys = mdlOutputs(t,x,u,parmodel,parcontroller) %#ok<INUSL>

%% Parse input and constants also defined in parcontroller
kpsi       = parcontroller.kpsi; % Gain for psi control
ktauxy     = parcontroller.ktauxy; % Gain for rz control
MomInertia = parmodel.MomInertia;
mass       = parmodel.mass;
satT       = parcontroller.satT;
aref       = u(1:3);
Psiref     = u(4);
rx         = u(5:7);
ry         = u(8:10);
rz         = u(11:13);
w           = u(14:16);

%% Control law for the torque ? and thurst T
[negpsi,~,~] = dcm2angle( [rx ry rz], 'ZYX' ); psi = -negpsi;

wxref = -ry'*aref;
wyref = rx'*aref;
wzref = (Psiref-psi);

tauxy = ktauxy*MomInertia(1:2,1:2)*([wxref wyref]'-w(1:2));
taupsi = kpsi*MomInertia(3,3)*(wzref-w(3));
tau = [tauxy;taupsi];

tau_ = tau +[0 -w(3) w(2); 
w(3) 0 -w(1);
-w(2) w(1) 0]*(MomInertia*w) ;
T_ = mass*norm(aref,2);   

Tlowerlimit = satT; 
if T_ > Tlowerlimit
     T = T_;
else
    T = Tlowerlimit;
end

%% Output
sys = [T; tau_];


