function addjartopath()
    % Add .jar file to static path
    javapath = fullfile(prefdir, 'javaclasspath.txt');
    jarpath = fullfile(pwd, 'm\websocket\jar\matlab-websocket-1.4.jar');

    fileID = fopen(javapath, 'w');
    fprintf(fileID, '%s', jarpath);
    fclose(fileID);

    % Restart MATLAB
    !matlab & 
    exit
end