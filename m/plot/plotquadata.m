%% Plot the simulation data 
% Distills the information in simlogdata resulting from a simulation to 
% plot the state and input. Control options allow to plot only a subset of
% the states and inputs.

close all;

%% Choose what to plot
option.plot_position = true; % Plot position
option.plot_velocity = true; % Plot velocity
option.plot_lambda   = true; % Plot lambda
option.plot_omega    = true; % Plot omega
option.plot_u        = true; % Plot u

%% Plot position
if option.plot_position
    figure(1);
    subplot(3,1,1);
    plot(simlogdata.time,simlogdata.signals.values(:,1));
    hold on;
    plot(retraj.t,reftraj.x), legend({'x','ref x'}), xlabel('time'), ylabel('x');
    grid on;
    subplot(3,1,2);
    plot(simlogdata.time,simlogdata.signals.values(:,2));
    hold on;
    plot(retraj.t,reftraj.y), legend({'y','ref y'}), xlabel('time'), ylabel('y');
    grid on;
    subplot(3,1,3);
    plot(simlogdata.time,simlogdata.signals.values(:,3));
    hold on;
    plot(retraj.t,reftraj.z), legend({'z','ref z'}),  xlabel('time'), ylabel('z');
    grid on;
    set(1,'Position',[171 467 549 338]);
    subplot(3,1,1), title('Position');
end

%% Plot velocity
if option.plot_velocity
    figure(2);
    subplot(3,1,1);
    plot(simlogdata.time,simlogdata.signals.values(:,4));
    hold on;
    plot(retraj.t,reftraj.vx), legend({'vx','ref vx'}), xlabel('time'), ylabel('vx');
    grid on;
    subplot(3,1,2);
    plot(simlogdata.time,simlogdata.signals.values(:,5));
    hold on;
    plot(retraj.t,reftraj.vy), legend({'vy','ref vy'}), xlabel('time'), ylabel('vy');
    grid on;
    subplot(3,1,3);
    plot(simlogdata.time,simlogdata.signals.values(:,6));
    hold on;
    plot(retraj.t,reftraj.vz), legend({'vz','ref vz'}),  xlabel('time'), ylabel('vz');
    grid on;
    set(2,'Position',[721 470 547 335]);
    subplot(3,1,1), title('Velocity');
end

%% Plot lambda
if option.plot_lambda
    figure(3);
    subplot(3,1,1);
    plot(simlogdata.time,simlogdata.signals.values(:,7));
    legend({'\phi'}), xlabel('t'), ylabel('\phi');
    grid on;
    subplot(3,1,2);
    plot(simlogdata.time,simlogdata.signals.values(:,8));
    legend({'\theta'}), xlabel('t'), ylabel('\theta');
    grid on;
    subplot(3,1,3);
    plot(simlogdata.time,simlogdata.signals.values(:,9));
    hold on;
    plot(retraj.t,reftraj.psi), legend({'\psi','psiref'}),  xlabel('t'), ylabel('\psi');
    grid on;
    set(3,'Position',[171 55 548 338]);
    subplot(3,1,1), title('Euler angles');
end

%% Plot omega
if option.plot_omega
    figure(4);
    subplot(3,1,1);
    plot(simlogdata.time,simlogdata.signals.values(:,10));
    legend({'\phi dot'}), xlabel('t'), ylabel('\phi dot');
    grid on;
    subplot(3,1,2);
    plot(simlogdata.time,simlogdata.signals.values(:,11));
    legend({'\theta dot'}), xlabel('t'), ylabel('\theta dot');
    grid on;
    subplot(3,1,3);
    plot(simlogdata.time,simlogdata.signals.values(:,12));
    hold on;
    plot(retraj.t,reftraj.vpsi), legend({'\psi dot','psiref'}),  xlabel('t'), ylabel('\psi dot');
    grid on;
    set(4,'Position',[721 58 548 338]);
    subplot(3,1,1), title('Angular velocities');
end

%% Plot u
if option.plot_u
    figure(5);
    subplot(4,1,1);
    plot(simlogdata.time,simlogdata.signals.values(:,13));
    legend({'thrust T'}), xlabel('t'), ylabel('thrust T');
    grid on;
    subplot(4,1,2);
    plot(simlogdata.time,simlogdata.signals.values(:,14));
    legend({'torque \tau\phi'}), xlabel('t'), ylabel('\tau\phi');
    grid on;
    subplot(4,1,3);
    plot(simlogdata.time,simlogdata.signals.values(:,15));
    legend({'torque \tau\theta'}), xlabel('t'), ylabel('\tau\theta');
    grid on;
    subplot(4,1,4);
    plot(simlogdata.time,simlogdata.signals.values(:,16));
    legend({'torque \tau\psi'}), xlabel('t'), ylabel('\tau\psi');
    grid on;
    set(5,'Position',[721 58 548 338]);
    subplot(4,1,1), title('Torques');
end