function [] = Disconnect(handle)
%DISCONNECT Summary of this function goes here
%   Detailed explanation goes here
fclose(handle)
disp("Disconnected!")
end

