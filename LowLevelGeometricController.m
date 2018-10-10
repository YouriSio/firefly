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

switch flag,
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  % Initialize the states, sample times, and state ordering strings.
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  % Return the outputs of the S-function block.
  case 3
    sys=mdlOutputs(t,x,u,parmodel,parcontroller);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  % There are no termination tasks (flag=9) to be handled.
  % Also, there are no continuous or discrete states,
  % so flags 1,2, and 4 are not used, so return an empty
  % matrix 
  case { 1, 2, 4, 9 }
    sys=[];

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Unexpected flags (error handling)%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Return an error message for unhandled flag values.
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end timestwo

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;  
sizes.NumInputs      = 16;  
sizes.DirFeedthrough = 1;   % has direct feedthrough
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
x0  = [];
ts  = [-1 0];   % inherited sample time

% specify that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u,parmodel,parcontroller)

%% parse input and constants also defined in parcontroller
kpsi       = parcontroller.kpsi; % gain for psi control
ktauxy     = parcontroller.ktauxy; % gain for rz control
MomInertia = parmodel.MomInertia;
mass       = parmodel.mass;
satT       = parcontroller.satT;
aref       = u(1:3);
Psiref     = u(4);
rx         = u(5:7);
ry         = u(8:10);
rz         = u(11:13);
w           = u(14:16);

%% control law for the torque \tau and thurst T
[negpsi,~,~] = dcm2angle( [rx ry rz], 'ZYX' ); psi = -negpsi;
% [psi,theta,phi] = dcm2angle( rotx(0)*roty(30)*rotz(45), 'ZYX' ) gives negative angles
% R = angle2dcm(psi,theta,phi,'ZYX')

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

Tlowerlimit = satT; % saturation
if T_ > Tlowerlimit
     T = T_;
else
    T = Tlowerlimit;
end

% output
sys = [T_; tau_];


