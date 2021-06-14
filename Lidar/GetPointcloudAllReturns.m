function [Pointcloud] = GetPointcloudAllReturns(TCPhandle,PacketsPerScan)
%GETPOINTCLOUD Summary of this function goes here
%   Detailed explanation goes here

    % vertical angle config
    AngConfig=[3.2,0,-3.04,-6.08,-9.13,-12.17,-15.21,-18.25];
    
    % populate vertical angle for sph2cart
    vertAng=repmat(AngConfig,50*PacketsPerScan,1);

    % fetch packets
    [~,RangeBearingAll,Intensity] = ParsePacketAllReturns(TCPhandle,PacketsPerScan);
    
    % convert range/bearing (aka spherical coordinates) to Cartesian
    [x1,y1,z1] = sph2cart(RangeBearingAll(:,1)*pi/180,vertAng*pi/180,RangeBearingAll(:,2:9));
    [x2,y2,z2] = sph2cart(RangeBearingAll(:,1)*pi/180,vertAng*pi/180,RangeBearingAll(:,10:17));
    [x3,y3,z3] = sph2cart(RangeBearingAll(:,1)*pi/180,vertAng*pi/180,RangeBearingAll(:,18:25));
    
    % convert points to MATLAB pointcloud object
    x=[reshape(x1,[PacketsPerScan*50*8,1]);reshape(x2,[PacketsPerScan*50*8,1]);reshape(x3,[PacketsPerScan*50*8,1])];
    y=[reshape(y1,[PacketsPerScan*50*8,1]);reshape(y2,[PacketsPerScan*50*8,1]);reshape(y3,[PacketsPerScan*50*8,1])];
    z=[reshape(z1,[PacketsPerScan*50*8,1]);reshape(z2,[PacketsPerScan*50*8,1]);reshape(z3,[PacketsPerScan*50*8,1])];
    TotalIntensity=[reshape(Intensity(:,1:8),[PacketsPerScan*50*8,1]);reshape(Intensity(:,9:16),[PacketsPerScan*50*8,1]);reshape(Intensity(:,17:24),[PacketsPerScan*50*8,1])];
    
    Pointcloud = pointCloud([x,y,z],'Intensity',TotalIntensity);

end

