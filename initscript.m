%% Initialization script
% loads the trajectory T in T.m generated by clicks2traj, 
% sets the simulation time, and defines the constants which parameterize 
% the controller and the quad model, defined in parcontroller and parmode, respectively.
% This script is run in the clicks2traj.m but it can also be run independently when a
% trajectory is alread in T.m.

%% Load T
clear all
load T
load LEDdata

% -------------------------------------------------------------------------
%% simulation parameters
simTime    = floor(length(T.X(1,:))/T.period);
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
%% initial conditions and parameters for model
parmodel.MomInertia = diag([0.002 0.002 0.004]); % moment of inertia
parmodel.mass       = 0.5;                       % mass
parmodel.g          = 9.81;                      % gravity
parmodel.pose0      = [T.X(1,1) T.Y(1,1) T.Z(1,1) T.PSI(1,1)]; % initial quad position and yaw
parmodel.cdrag      = 0.1;                       % drag coefficient
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
%% parameters for the controller
parcontroller.k1 = 1.5;      % Velocity gain high level trajectory tracking
parcontroller.k2 = 1.5;      % Position D high level trajectory tracking
parcontroller.k3 = 0.6;      % Position P high level trajectory tracking
parcontroller.kpsi = 3;    % gain for psi control low level control
parcontroller.ktauxy  = 100; % gain for rz control low level control
parcontroller.satT = 0.1;    % Thurst saturation sat
parcontroller.feedforward= 1; % 1 use feedforward, 0 do not
% -------------------------------------------------------------------------
