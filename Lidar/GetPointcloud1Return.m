function [Pointcloud] = GetPointcloud1Return(TCPhandle,PacketsPerScan)
%GETPOINTCLOUD Summary of this function goes here
%   Detailed explanation goes here

    % vertical angle config
    AngConfig=[3.2,0,-3.04,-6.08,-9.13,-12.17,-15.21,-18.25];
    
    % populate vertical angle for sph2cart
    vertAng=repmat(AngConfig,50*PacketsPerScan,1);

    % fetch packets
    [~,RangeBearingAll,Intensity] = ParsePacket1Return(TCPhandle,PacketsPerScan);
    
    % convert range/bearing (aka spherical coordinates) to Cartesian
    [x,y,z] = sph2cart(RangeBearingAll(:,1)*pi/180,vertAng*pi/180,RangeBearingAll(:,2:9));
    
    % convert points to MATLAB pointcloud object
    x=reshape(x,[PacketsPerScan*50*8,1]);
    y=reshape(y,[PacketsPerScan*50*8,1]);
    z=reshape(z,[PacketsPerScan*50*8,1]);
    Intensity=reshape(Intensity,[PacketsPerScan*50*8,1]);
    
    Pointcloud = pointCloud([x,y,z],'Intensity',Intensity);

end

