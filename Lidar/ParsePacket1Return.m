function [Timestamp,RangeBearing,Intensity] = ParsePacket1Return(TCPhandle,PacketsPerScan)
%PARSEPACKET Summary of this function goes here
% 
% info packet [1]
    %bytes
    %1-4 uint32 Packet Signature [Ox75BD7e97]
    %5-8 uint32 Message Size [bytes]
    %9-12 uint32 Timestamp seconds [sec]
    %13-16 uint32 Timestamp Nanoseconds [sec*10^-9]
    %17 uint8 API Version Major [0]
    %18 uint8 API Version Minor [1]
    %19 uint8 API Version Patch [0]
    %20 uint8 Packet Type [0x00(all returns)] 
    
%main packet [50] 21-6620(50*132 bytes in total)
    %uint16 Encoder position [0-10399]
    %uint16 Reserved [0]
    %uint32 Distance(3*8) [10 micrometers] 
    %uint8 Intensities(3*8) [0-255]
    %uint8 Return status per laser(8) [0(no error)]
    
% status packet [1] 6621-6632
    %uint32 Timestamp seconds [sec]
    %uint32 Timestamp Nanoseconds [sec*10^-9]
    %uint16 APIVersion2 [5]
    %uint16 Status [0(no error)]
 %total bytes per packet 4*4+1*4+50*(2*2+4*3*8+1*3*8+1*8)+4*2+2*2 = 6632 bytes 
 
    Packet = TCPhandle.UserData; %Unload packet data
    PacketSize = 6632; %bytes/uint8
    PackeCount = PacketsPerScan; %number of packets concatenated
    
    %-----------------------------------------
    %parse packet
    %-----------------------------------------
    % init vars
	Intensity=zeros(50*PackeCount,8); 
	RangeBearing=zeros(50*PackeCount,1+8);
    Timestamp=zeros(PackeCount,1);
    
    % header
    for i=0:PackeCount-1
        PacketInd = i*PacketSize;
        % for function debugging
        % Signature = dec2hex(typecast(flip(Packet(1+PacketInd:4+PacketInd),2),'uint32'));
        % MessageSize = typecast(flip(Packet(5+PacketInd:8+PacketInd),2),'uint32');
        
        %get timestamp
        TimestampSec = double(typecast(flip(Packet(9+PacketInd:12+PacketInd),2),'uint32'));
        TimestampNano = double(typecast(flip(Packet(13+PacketInd:16+PacketInd),2),'uint32'));
        Timestamp(i+1) = TimestampSec+TimestampNano*10^-9;

        % main packet
        for i2=1:50 %process 132 bytes per loop iteration
            %uint16 Encoder position [0-10399]
            EncoderPos = double(typecast(flip(Packet(132*(i2-1)+21+PacketInd:132*(i2-1)+22+PacketInd),2),'uint16'));
            RangeBearing(i*50+i2,1) =(EncoderPos/10400)*360; %enc pos to deg;

            %uint32 Distance(8) [10 micrometers]
            % fliplr() is bad, use flip(x,2) directly for much greater speed (which is what MATLAB is using internally anyways).

            RangeBearing(i*50+i2,2:9) = double(typecast(flip(Packet(132*(i2-1)+25+PacketInd:132*(i2-1)+56+PacketInd),2),'uint32')).*10^-5;

            %uint8 Intensities(8) [0-255]
            Intensity(i*50+i2,1:8) = Packet(132*(i2-1)+121+PacketInd:132*(i2-1)+128+PacketInd);

        end
    end
end

