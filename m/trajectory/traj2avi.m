%% Function trajectory2avi
% This function will take a trajectory and convert it to video format so
% the trajectory can be analysed.

function traj2avi (ID, traj)
    % Find the recording dimensions
    record_dim = evalin('base', 'plots.record_dim');
    
    % Load the VR world and quad node
    world = vrworld(fullfile(pwd, 'wrl\NeutralSingleDroneLEDs.wrl'));
    open(world);
    quad = world.Quad1;


    % Create the figure
    f = vrfigure(world);
    set(f, 'Name', strcat('(', ID, ')', ' Actual trajectory'));
    set(f, 'Viewpoint', 'West');
    set(f, 'Antialiasing', 'on');
    set(f, 'NavPanel', 'none');
    set(f, 'Position', [0 0 record_dim(1) record_dim(2)]);

    % Recording settings
    set(world, 'RecordInterval',[traj.t(1) traj.t(end)]);
    set(f, 'Record2DFileName', fullfile(pwd, 'data', ID, strcat(ID, '.avi')));
    set(f, 'Record2D', 'on');
    set(f, 'Record2DFPS', size(traj.t,2) / traj.t(end));
    set(f, 'Record2DCompressQuality', 100);

    set(world, 'RecordMode', 'scheduled');

    % Continuously show the quad's movements
    for i=1:size(traj.t,2)
        % Set the new time
        set(world, 'Time', traj.t(i));

        % Set the rotation and translation
        quad.translation = [-traj.x(i) traj.y(i) traj.z(i)];

        euler_angles = [traj.psi(i)+pi/2 traj.phi(i) traj.theta(i)];
        rotation_matrix = eul2rotm(euler_angles);
        axisangle_rotation = rotm2axang(rotation_matrix);

        quad.rotation = axisangle_rotation;

        % Update the figure
        vrdrawnow;
    end

    % Close the figure
    close(f);
end