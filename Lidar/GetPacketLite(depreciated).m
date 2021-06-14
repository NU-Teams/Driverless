function [Timestamp,FiringData,FiringDataIntensities] = GetPacketLite(TCPhandle)
%READLIDAR Summary of this function goes here
%   WARNING, FOR VIEWING ONLY

% info packet [1]
    %bits 
    %1-32 uint32 IGNORED
    %33-64 uint32 IGNORED
    %65-96 uint32 IGNORED
    %97-128 uint32 IGNORED
    %129-136 uint8 IGNORED
    %137-144 uint8 IGNORED
    %145-152 uint8 IGNORED
    %153-160 uint8 IGNORED
    
%main packet [50] 
    %uint16 Encoder position [0-10399]
    %uint16 IGNORED
    %uint32 Distance(3*8) [10 micrometers] 
    %uint8 Intensities(3*8) [0-255]
    %uint8 IGNORED
    
% status packet [1] 
    %uint32 Timestamp seconds [sec]
    %uint32 Timestamp Nanoseconds [sec*10^-9]
    %uint16 IGNORED
    %uint16 IGNORED
 %total bytes per packet 4*4+1*4+50*(2*2+4*3*8+1*3*8+1*8)+4*2+2*2 = 6632 bytes  
 
    
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
    read(TCPhandle,5,'uint32');%skip first 160 bits
    
    FiringData=zeros(50,25);
    FiringDataIntensities=zeros(50,24);
    %main packet (scan data) aka the stuff that matters
    for i2=1:50
        %position
        FiringData(i2,1)=read(TCPhandle,1,'uint16');
        %reserved
        read(TCPhandle,1,'uint16');
        %distance
        FiringData(i2,2:24+1)=read(TCPhandle,24,'uint32'); 
        FiringDataIntensities(i2,1:24)=read(TCPhandle,24,'uint8');

        read(TCPhandle,8,'uint8');
    end

    Sec=read(TCPhandle,1,'uint32');
    Nano=read(TCPhandle,1,'uint32');
    Timestamp=Sec+Nano*10^-9;
    read(TCPhandle,2,'uint16');
    
    %minor pre-processing
    FiringData(:,1)=(FiringData(:,1)./10400).*360; %enc pos to deg
    FiringData(:,2:25)=FiringData(:,2:25).*10^-5; %distance to meter
    

end

