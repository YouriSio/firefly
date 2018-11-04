%% Function points2actualtraj
% Converts waypoints to an actually simulted trajectory.

function [reftraj, actualtraj] = points2actualtraj(ID, waypoints)
    % Create the directory for data
    mkdir(fullfile(pwd, 'data', ID));
    
    %% Save the waypoints
    t   = waypoints(:,1)';
    x   = waypoints(:,2)';
    y   = waypoints(:,3)';
    z   = waypoints(:,4)';
    psi = waypoints(:,5)';
    
    save(fullfile(pwd, 'data', ID, 'waypoints'), 't', 'x', 'y', 'z' ,'psi');
    
    %% Generate the reference trajectory for the simulation
    reftraj = points2reftraj(waypoints);
    
    % Save the reference trajectory
    save(fullfile(pwd, 'data', ID, 'reftraj'), 'reftraj');
    
    %% Generate the actual trajectory
    pose0 = [reftraj.x(1,1), reftraj.y(1,1), reftraj.z(1,1), reftraj.psi(1,1)];
    
    assignin('base', 'temp', pose0);          % This is bad.... asigning to base workspace
    evalin('base', 'parmodel.pose0 = temp;'); % So should be fixed
    
    assignin('base', 'reftraj', reftraj);
    
    sim_time = reftraj.t(1,end);
    sim('simexample', sim_time);
    
    % Extract actual trajectory from the simulation log data
    actualtraj.t      = simlogdata.time';
    actualtraj.x      = simlogdata.signals.values(:,1)';
    actualtraj.y      = simlogdata.signals.values(:,2)';
    actualtraj.z      = simlogdata.signals.values(:,3)';
    actualtraj.vx     = simlogdata.signals.values(:,4)';
    actualtraj.vy     = simlogdata.signals.values(:,5)';
    actualtraj.vz     = simlogdata.signals.values(:,6)';
    actualtraj.phi    = simlogdata.signals.values(:,7)';
    actualtraj.theta  = simlogdata.signals.values(:,8)';
    actualtraj.psi    = simlogdata.signals.values(:,9)';
    actualtraj.vphi   = simlogdata.signals.values(:,10)';
    actualtraj.vtheta = simlogdata.signals.values(:,11)';
    actualtraj.vpsi   = simlogdata.signals.values(:,12)';
    
    %% Save the actual trajectory
    save(fullfile(pwd, 'data', ID, 'actualtraj'), 'actualtraj');
    
    if evalin('base', 'plots.plot_trajectories')
        clf;
        imshow './img/soccerfieldpic.jpg'
        field_dim = evalin('base', 'plots.field_dim');
        plot3(x, y, z);
        hold on;
        plot3(reftraj.x, reftraj.y, reftraj.z);
        hold on;
        plot3(actualtraj.x, actualtraj.y, actualtraj.z);
        axis equal;
        grid on;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        legend('Waypoints', 'Reference trajecotry', 'Actual trajectory');
        view(-70,40);
        Xsf = [field_dim(1), field_dim(1), -field_dim(1), -field_dim(1)]/2;
        Ysf = [field_dim(2), -field_dim(2), -field_dim(2),  field_dim(2)]/2;
        patch(Xsf,Ysf,[0 0 0 0],[0.1 1 0.1]);
    end
end