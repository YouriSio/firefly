%% Starts a websocket server
% The actual code that handles the data is in websocket.m
function startserver()
    port = 30000;

    % Check if .jar file has been added to the java static path, else install
    try
        WebSocket(port);
    catch ME
        if (ME.identifier == "MATLAB:UndefinedFunction")
            % Display warning message
            warning('Adding websocket jar file to java static path');

            % Add .jar file to java static path
            addjartopath();
        else
            warning('An unknown error occured');
        end
    end
end