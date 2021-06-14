% export ROS_MASTER_URI=http://192.168.42.131:11311 >> ~/.bashrc
% export ROS_IP=192.168.42.131 >> ~/.bashrc
% source ./devel/setup.bash
% roslaunch eufs_launcher eufs_launcher.launch

% rosgenmsg('..\Gazebosim'); %Matlab only supports [Microsoft Visual C++ 2017 product family] for ROS toolbox
rosshutdown;

pyenv
rosinit('http://192.168.42.131:11311','NodeHost','192.168.42.100')

% [pub,msg]=rospublisher('/ros_can/mission_flag')
% msg.Data=true;
% send(pub,msg)

[pub,msg]=rospublisher('/cmd_vel_out')
msg.Drive.Acceleration=5;
msg.Drive.Speed=30;
send(pub,msg)

coneSub = rossubscriber('/imu')
receive(coneSub)