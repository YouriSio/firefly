% Add .jar file to static path
javapath = fullfile(prefdir, 'javaclasspath.txt');
jarpath = fullfile(pwd, 'jar', 'matlab-websocket-1.4.jar');

fileID = fopen(javapath, 'w');
fprintf(fileID, '%s', jarpath);
fclose(fileID);

% Restart MATLAB
!matlab & 
exit