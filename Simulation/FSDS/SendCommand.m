function [] = SendCommand(handle,Throttle,Steering,Brake)
%SENDCOMMAND Summary of this function goes here
%   Detailed explanation goes here
fprintf(handle,"CTRL");
fprintf(handle,string(Throttle)+" "+string(Steering)+" "+string(Brake));
end

