function [packet] = GetPacketFull(handle)
%#codegen 
% WARNING, FOR VIEWING ONLY

% info packet [1]
    %uint32 Packet Signature [Ox75BD7e97]
    %uint32 Message Size [bytes]
    %uint32 Timestamp seconds [sec]
    %uint32 Timestamp Nanoseconds [sec*10^-9]
    %uint8 API Version Major [0]
    %uint8 API Version Minor [1]
    %uint8 API Version Patch [0]
    %uint8 Packet Type [0x00(all returns)] 
    
%main packet [50]
    %uint16 Encoder position [0-10399]
    %uint16 Reserved [0]
    %uint32 Distance(3*8) [10 micrometers] 
    %uint8 Intensities(3*8) [0-255]
    %uint8 Return status per laser(8) [0(no error)]
    
% status packet [1]
    %uint32 Timestamp seconds [sec]
    %uint32 Timestamp Nanoseconds [sec*10^-9]
    %uint16 APIVersion2 [5]
    %uint16 Status [0(no error)]
    
%% vertical beam deg
% 3.2 [deg]
% 0 [deg]
% -3.04 [deg]
% -6.08 [deg]
% -9.13 [deg]
% -12.17 [deg]
% -15.21 [deg]
% -18.25 [deg]

%% parse packet
% info packet
    packet.PacketSignature=fread(handle,1,'uint32');% execute dec2hex() for hex code (but its resource intensive)
    packet.MessageSize=fread(handle,1,'uint32');
    Sec=fread(handle,1,'uint32');
    Nano=fread(handle,1,'uint32');
    packet.Timestamp1=Sec+Nano*10^-9;
    packet.APIVersion=fread(handle,3,'uint8');
    packet.PacketType=fread(handle,1,'uint8');

    %main packet (scan data) aka the stuff that matters
    packet.FiringData=zeros(50,25);
    packet.FiringDataIntensities=zeros(50,24);
    packet.FiringDataStatus=zeros(50);
    for i2=1:50
        %position
        packet.FiringData(i2,1)=(fread(handle,1,'uint16')/10400)*360; %enc pos to deg
        %reserved
        fread(handle,1,'uint16');
        %distance
        packet.FiringData(i2,2:24+1)=fread(handle,24,'uint32')'*10^-5; %to meter
        packet.FiringDataIntensities(i2,1:24)=fread(handle,24,'uint8')';
        packet.FiringDataStatus(i2,1:8)=fread(handle,8,'uint8');
    end

    Sec=fread(handle,1,'uint32');
    Nano=fread(handle,1,'uint32');
    packet.Timestamp2=Sec+Nano*10^-9;
    packet.APIVersion2=fread(handle,1,'uint16');
    packet.Status=fread(handle,1,'uint16');
end

