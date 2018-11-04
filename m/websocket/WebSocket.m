classdef WebSocket < WebSocketServer
    % WebSocket opens a websocket and communicates with the clients
    
    properties
        backlog = {}
    end
    
    methods (Access = protected)
            
        function onOpen(obj,conn,message)   
        end
        
        function onTextMessage(obj,conn,message)
            % Confirm the sender the information was received
            obj.sendTo(conn.HashCode, 'I have received your points!');
            
            % Reshape the string into a matrix  with columns time, x, y, z, phi
            trajectoryData = reshape(str2num(message),[],5);
            
            % Store the trajectorydata with the associated client in a
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
            
            % Check if the trajectory is valid
            [valid, actualtraj] = checktrajectory(coords);
        end
    end
end

