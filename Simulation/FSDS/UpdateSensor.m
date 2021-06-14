function [data] = UpdateSensor(handle)
%GET_IMU Summary of this function goes here
%   Detailed explanation goes here
fprintf(handle,"UPDATE");

%% IMU and LIDAR

packet=fscanf(handle);
packet=replace(packet,', [','],[');
packet=str2num(packet);
data.imu.time_stamp=packet(1);%in nanosec
data.imu.orientation.x=packet(2);%relative to north pole (compass)
data.imu.orientation.y=packet(3);
data.imu.orientation.z=packet(4);
data.imu.angular_velocity.x=packet(5);%in rad/s
data.imu.angular_velocity.y=packet(6);
data.imu.angular_velocity.z=packet(7);
data.imu.linear_acceleration.x=packet(8);%in ms^-2
data.imu.linear_acceleration.y=packet(9);
data.imu.linear_acceleration.z=packet(10);
% unpack GPS data
data.gps.time_stamp=packet(11);
data.gps.time_utc=packet(12);
data.gps.geo_point.latitude=packet(13);
data.gps.geo_point.longitude=packet(14);
data.gps.geo_point.altitude=packet(15);
data.gps.eph=packet(16);
data.gps.epv=packet(17);
data.gps.velocity.x=packet(18);
data.gps.velocity.y=packet(19);
data.gps.velocity.z=packet(20);
% unpack lidar pointcloud
data.lidar.time_stamp=packet(21);%in nanosec
packet(1:21)=[];
data.lidar.pointcloud=reshape(packet,3,length(packet)/3)';


%% LIDAR
% tic
% % lidarTemp=fscanf(handle);
% lidarTemp=replace(lidarTemp,', [','],[');
% lidarTemp=str2num(lidarTemp);
% data.lidar.time_stamp=lidarTemp(1);%in nanosec
% lidarTemp(1)=[];
% data.lidar.pointcloud=reshape(lidarTemp,3,length(lidarTemp)/3)';%in pointcloud array [x y z]
% toc
%% Camera

camTemp=uint8(fread(handle,785*785*3+1));

camTemp=camTemp(2:length(camTemp));
camTemp=permute(reshape(camTemp,3,785,785),[3 2 1]);

%revert R,B channel
img(:,:,1)=camTemp(:,:,3);
img(:,:,2)=camTemp(:,:,2);
img(:,:,3)=camTemp(:,:,1);

data.camera.image=img;

end

