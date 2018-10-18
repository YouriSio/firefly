classdef WebSocket < WebSocketServer
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        clients = {}
    end
    
    methods (Access = protected)
            
        function onOpen(obj,conn,message)   
        end
        
        function onTextMessage(obj,conn,message)   
            trajectoryData = reshape(str2num(message),[],5);
            client = {conn, trajectoryData};
            obj.clients{end + 1} = client;
            obj.sendTo(conn.HashCode, 'I have received your points!');
            obj.createTrajectory(client);
        end
        
        function onBinaryMessage(obj,conn,message)
        end
        
        function onError(obj,conn,message)
        end
        
        function onClose(obj,conn,message)
        end
        
        function createTrajectory(obj,client)
            coords = client{2};
            time = coords(:,1);             % Discrete-time
            px   = coords(:,2);             % X  reference
            py   = coords(:,3);             % Y  reference
            pz   = coords(:,4);             % Z  reference
            psi  = coords(:,5);             % Yaw reference
            
            save('./mat/waypoints', 'time', 'px', 'py', 'pz', 'psi');
            run('clicks2traj.m');
        end
    end
end

