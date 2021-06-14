function [x,y,z] = GetPoints(TCPhandle,PacketsPerScan)
%GETPOINTS Summary of this function goes here
%   Detailed explanation goes here


    AngConfig=[3.2,0,-3.04,-6.08,-9.13,-12.17,-15.21,-18.25];
	vertAng=repmat(AngConfig,50*PacketsPerScan,1);

    [Timestamp,RangeBearingAll,Intensity] = ParsePacket1Return(TCPhandle);
    
    % convert range/bearing (aka spherical coordinates) to Cartesian
    [x,y,z] = sph2cart(RangeBearingAll(:,1)*pi/180,vertAng*pi/180,RangeBearingAll(:,2:9));
end

