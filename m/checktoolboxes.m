%% Checks if necessary toolboxes are installed
% This function is called with the run file and makes sure all the necessary
% toolboxes are installed. If not it will return an error with the name of
% the toolbox not installed.
function checktoolboxes()
add_ons =  ["Simulink","Robotics System Toolbox","Aerospace Toolbox","Aerospace Blockset","Phased Array System Toolbox","Simulink 3D Animation"];
installed = struct2table(ver());
installed = installed(:,1);
installed = table2array(installed); 

for i = 1:length(add_ons)
    if any(strcmp(installed,add_ons(i)))
        fprintf('Found %s.\n',add_ons(i))
    else
       error('Could not find %s, make sure it is properly installed and try again.\nShutting down...',add_ons(i));
    end
end

