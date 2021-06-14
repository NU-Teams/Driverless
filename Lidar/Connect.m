function [TCPhandle] = Connect(IP,Returns,PacketsPerScan)
%Connect Summary of this function goes here
%   connects to the MQ8 Lidar
%   4141 is the default port

    TCPhandle= tcpclient(IP,4141,"Timeout",3,"ConnectTimeout",5); 
    TCPhandle.ByteOrder = "big-endian";
    
    %configure callback function that reads and clears buffer in parallel
    switch Returns
        case "full"
            configureCallback(TCPhandle,"byte",6632*PacketsPerScan,@TCPCallbackFull);
            disp('Connected!')
        case "reduced"
            configureCallback(TCPhandle,"byte",2224*PacketsPerScan,@TCPCallbackReduced);
            disp('Connected!')
        otherwise
            diso('invalid return mode, must be "full" or "reduced" depending on Lidar setting')
    end
end

