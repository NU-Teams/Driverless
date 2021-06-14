% This is used to connect to the MQ8 Lidar and gather data.
% date: 22/4/2021
% by: Kwan Nok Mak (3250260)
%
% to configure sensor, open web browser, enter IP with port:7780. 
% Username: editor
% Password: qeditor

clear
clc
%% Add helper functions to path
% addpath Functions/Detection
%-----------------------------------------------------------------------------------
%% Connect to Lidar
% pick a reasonable number that covers 360deg depending on lidar setting
% high number = higher scan resolution but also more delay. 
PacketsPerScan = 220; % remember to change the numbers in the callback function

TCPhandle=Connect('192.168.1.1',"full",PacketsPerScan);
pause(15);
%-----------------------------------------------------------------------------------
%% Read Lidar
	
player = pcplayer([-10 10],[-10 10],[-10 10]);


while isOpen(player) 
     ptCloud = GetPointcloudAllReturns(TCPhandle,PacketsPerScan);
     view(player,ptCloud);           
end
%-----------------------------------------------------------------------------------
%% Disconnect
% clear TCPhandle
% disp("Disconnected!")
