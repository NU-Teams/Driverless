function handle = Connect()
%CONNECT Summary of this function goes here
%   Detailed explanation goes here
handle= tcpip('127.0.0.1',12201)
handle.InputBufferSize=2500000;
handle.Timeout=5;
handle.Terminator = ']';
fopen(handle);
disp('Connected!')
end

