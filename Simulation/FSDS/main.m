clc
clear
close
format shorte

system('start StartServer1.bat') %this will start the python server which interperts the fsds api for us
pause(5);

%% start connection (TCP/IP)
socket=Connect();
plots=InitOverview();

for i=1:100

sensors=UpdateSensor(socket);
SendCommand(socket,0.2,1,0);
UpdateOverview(plots,sensors);

end

Disconnect(socket);