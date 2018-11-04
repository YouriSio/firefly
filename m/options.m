%% Initialization script
% Loads the trajectory T in T.m generated by clicks2traj. Sets the 
% simulation time, and defines the constants which parameterize the 
% and the quad model, defined in parcontroller and parmode. This script is 
% run in the clicks2traj.m but it can also be run independently when a
% trajectory is already in T.m.

%% Initial conditions and parameters for model
parmodel.MomInertia = diag([0.002 0.002 0.004]);                % moment of inertia [kg m^2]
parmodel.mass       = 0.5;                                      % mass [kg]
parmodel.g          = 9.81;                                     % gravity [m/s^2]
parmodel.cfdrag     = 0.1;                                      % drag coefficient [-]
parmodel.cmdrag     = 0.0;                                      % rotation drag coefficient [-]

%% Parameters for the controller
parcontroller.k1          = 9;    % Velocity gain high level trajectory tracking
parcontroller.k2          = 1.5;  % Acceleration gain high level trajectory tracking
parcontroller.k3          = 0.6;  % Position P high level trajectory tracking
parcontroller.kpsi        = 3;    % Gain for psi control low level control
parcontroller.ktauxy      = 100;  % Gain for rz control low level control
parcontroller.feedforward = true; % Set feed forward

parcontroller.thrust_sat       = 0.1; % Define the minimal thrust
parcontroller.acceleration_sat = 10;  % Define the maximal absolute desired acceleration

%% Reference trajectory options
parcontroller.tau = 0.1;             % Input sampling period
parcontroller.N12 = 10;              % Ratio of input sampling period over output sampling period
parcontroller.rho = 0.01;           % Weighting factor for control input
parcontroller.constantheight = true; % Must drone stay at constant height
parcontroller.height = 1;            % The height in case above is true

%% Plot options
plots.plot_trajectories = true; % Choose to plot all 3 trajectories
plots.field_dim = [8 10];       % Dimensions of soccer field