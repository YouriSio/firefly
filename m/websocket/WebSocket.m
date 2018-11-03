classdef WebSocket < WebSocketServer
    %WebSocket opens a websocket and communicates with the clients
    
    properties
        backlog = {}
    end
    
    methods (Access = protected)
            
        function onOpen(obj,conn,message)   
        end
        
        function onTextMessage(obj,conn,message)
            % confirm the sender the information was received
            obj.sendTo(conn.HashCode, 'I have received your points!');
            
            % reshape the string into a matrix  with columns time, x, y, z, phi
            trajectoryData = reshape(str2num(message),[],5);
            
            % store the trajectorydata with the associated client in a
            % backlog
            client = {conn, trajectoryData};
            obj.backlog{end + 1} = client;
            
            % compute the trajectory
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
            
            % save the variables
            save(fullfile(pwd, 'mat', 'waypoints'), 'time', 'px', 'py', 'pz', 'psi');
            
            % compute the trajectory
            clicks2traj;
        end
    end
end

