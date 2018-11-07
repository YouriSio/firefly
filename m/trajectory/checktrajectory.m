%% Function checktrajectory
% Generates the actual trajectory through simulation and checks if the
% trajectory is valid.
function [actualtraj, valid, valid_msg] = checktrajectory(waypoints)
    %% Create an quasi-unique ID to store data
    ID = datestr(now,'yyyymmddHHMMSSFFF');
    fprintf('[ID = %s] Waypoint request received\n', ID);
    
    %% Simulate the actual trajectory
    fprintf('[ID = %s] Simulating actual trajectory...\n', ID);
    tic;
    [reftraj, actualtraj] = points2actualtraj(ID, waypoints);
    fprintf('[ID = %s] Done! Took %fs\n', ID, toc);
    
    %% Perform some checks on the actual trajectory to determine if OK
    fprintf('[ID = %s] Checking if trajectory is valid...\n', ID);
   
    % I propose to check 2 things here:
    % - Calculate MSE between ref and actual trajectory
    % - Check for collisions
    
    valid = true;
    
    % Check if z is ever lower than 0, this means collision with ground
    if (sum(actualtraj.z < 0) > 0)
        valid = false;
        valid_msg = 'Collision with ground\n';
        fprintf(2, '[ID = %s] The trajectory is not valid: %s\n', ID, valid_msg);
        return;
    end
    
    % Check if the drone ever leaves the football field
    field_dim = evalin('base', 'plots.field_dim');
    if (sum(abs(actualtraj.x) > field_dim(1)/2) > 0 || sum(abs(actualtraj.y) > field_dim(1)/2) > 0)
        valid = false;
        valid_msg = 'The drone went out of bounds';
        fprintf(2, '[ID = %s] The trajectory is not valid: %s\n', ID, valid_msg);
        return;
    end
    
    valid_msg = '';
    
    fprintf('[ID = %s] The trajectory is valid!\n', ID); 
end