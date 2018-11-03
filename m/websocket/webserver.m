%% Runs a websocket server
port = 30000;

% Check if .jar file has been added to the java static path, else install
try
    server = WebSocket(port);
catch ME
    if (ME.identifier == "MATLAB:UndefinedFunction")
        % Display warning message
        warning('Adding websocket jar file to java static path');
        
        % Add .jar file to java static path
        run(fullfile(pwd, 'm', 'server', 'addjartopath.m'));
    else
        warning('An unknown error occured');
    end
end