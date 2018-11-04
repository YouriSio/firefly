function [valid, reftraj, actualtraj] = checktrajectory(waypoints)
    %% Create an quasi-unique ID to store data
    ID = datestr(now,'yyyymmddHHMMSSFFF');
    fprintf('[ID = %s] Waypoint request received\n', ID);
    
    %% Simulate the actual trajectory
    fprintf('[ID = %s] Simulating actual trajectory...\n', ID);
    tic;
    [reftraj, actualtraj] = points2actualtraj(ID, waypoints);
    fprintf('[ID = %s] Done! Took %fs\n', ID, toc);
    
    %% Perform some checks on the actual trajectory to determine if OK
    fprintf('[ID = %s] Checking if trajectory is valid\n', ID);
   
    % I propose to check 2 things here:
    % - Calculate MSE between ref and actual trajectory
    % - Check for collisions
    
    valid = true;
end

