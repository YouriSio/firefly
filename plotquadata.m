%% Plot the simulation data 
% Distills the information in simlogdata resulting from a simulation to plot the state and input.
% Control options allow to plot only a subset of the states and inputs.

close all

optplotp      = 1; % 1, plot position, 0 - do not plot
optplotv      = 1; % 1, plot velocity, 0 - do not plot
optplotlambda = 1; % 1, plot lambda, 0 - do not plot
optplotomega  = 1; % 1, plot lambda, 0 - do not plot
optplotu      = 1; % 1, plot u = [T tauphi tautheta taupsi]

if optplotp
    figure(1)
    subplot(3,1,1)
    plot(simlogdata.time,simlogdata.signals.values(:,1))
    hold on
    plot(T.time,T.X), legend({'x','ref x'}), xlabel('time'), ylabel('x')
    grid on
    subplot(3,1,2)
    plot(simlogdata.time,simlogdata.signals.values(:,2))
    hold on
    plot(T.time,T.Y), legend({'y','ref y'}), xlabel('time'), ylabel('y')
    grid on
    subplot(3,1,3)
    plot(simlogdata.time,simlogdata.signals.values(:,3))
    hold on
    plot(T.time,T.Z), legend({'z','ref z'}),  xlabel('time'), ylabel('z')
    grid on
    set(1,'Position',[171 467 549 338])
    subplot(3,1,1), title('Position')
end

if optplotv
    figure(2)
    subplot(3,1,1)
    plot(simlogdata.time,simlogdata.signals.values(:,4))
    hold on
    plot(T.time,T.VX), legend({'vx','ref vx'}), xlabel('time'), ylabel('vx')
    grid on
    subplot(3,1,2)
    plot(simlogdata.time,simlogdata.signals.values(:,5))
    hold on
    plot(T.time,T.VY), legend({'vy','ref vy'}), xlabel('time'), ylabel('vy')
    grid on
    subplot(3,1,3)
    plot(simlogdata.time,simlogdata.signals.values(:,6))
    hold on
    plot(T.time,T.VZ), legend({'vz','ref vz'}),  xlabel('time'), ylabel('vz')
    grid on
    set(2,'Position',[721 470 547 335])
    subplot(3,1,1), title('Velocity')
end

if optplotlambda
    figure(3)
    subplot(3,1,1)
    plot(simlogdata.time,simlogdata.signals.values(:,7))
    legend({'phi'}), xlabel('time'), ylabel('phi')
    grid on
    subplot(3,1,2)
    plot(simlogdata.time,simlogdata.signals.values(:,8))
    legend({'theta'}), xlabel('time'), ylabel('theta')
    grid on
    subplot(3,1,3)
    plot(simlogdata.time,simlogdata.signals.values(:,9))
    hold on
    plot(T.time,T.PSI), legend({'psi','psiref'}),  xlabel('time'), ylabel('psi')
    grid on
    set(3,'Position',[171 55 548 338])
    subplot(3,1,1), title('Euler angles')
end

if optplotomega
    figure(4)
    subplot(3,1,1)
    plot(simlogdata.time,simlogdata.signals.values(:,10))
    legend({'phi dot'}), xlabel('time'), ylabel('phi dot')
    grid on
    subplot(3,1,2)
    plot(simlogdata.time,simlogdata.signals.values(:,11))
    legend({'theta dot'}), xlabel('time'), ylabel('theta dot')
    grid on
    subplot(3,1,3)
    plot(simlogdata.time,simlogdata.signals.values(:,12))
    hold on
    plot(T.time,T.VPSI), legend({'psi dot','psiref'}),  xlabel('time'), ylabel('psi dot')
    grid on
    set(4,'Position',[721 58 548 338])
    subplot(3,1,1), title('Euler angles')
end

if optplotu
    figure(5)
    subplot(4,1,1)
    plot(simlogdata.time,simlogdata.signals.values(:,13))
    legend({'thrust T'}), xlabel('time'), ylabel('thrust T')
    grid on
    subplot(4,1,2)
    plot(simlogdata.time,simlogdata.signals.values(:,14))
    legend({'torque tauphi'}), xlabel('time'), ylabel('tauphi')
    grid on
    subplot(4,1,3)
    plot(simlogdata.time,simlogdata.signals.values(:,15))
    legend({'torque tautheta'}),  xlabel('time'), ylabel('tautheta')
    grid on
    subplot(4,1,4)
    plot(simlogdata.time,simlogdata.signals.values(:,16))
    legend({'torque taupsi'}),  xlabel('time'), ylabel('taupsi')
    grid on
    set(5,'Position',[721 58 548 338])
    subplot(4,1,1), title('Euler angles')
end
