clear all;
clc;
clf

toa = 0:9; % time of arrival
course = 0;
r = 0;
t = 1;
if course == 0
               % Time,  Waypoint(x,y,z),Orientation
trajectoryInfo = [0*t,    0,-5,0,         0,0,0; ... % Initial position
                  0.5*t,  -0.5,-0.5,0,    0,0,0; ...
                  3*t,    1,6,0,          0,0,0; ...
                  3.5*t,  3,4,0,          0,0,0; ...
                  4*t,  5,5,0,          0,0,0; ...
                  4.2*t,  6,5,0,          0,0,0; ...
                  5*t,  10,5,0,         0,0,0; ...
                  6*t,    15,5,0,         0,0,0; ...
                  7*t,  12,2,0,         0,0,0; ...
                  8*t,    8,0,0,          0,0,0; ...
                  9*t,  11,-2,0,        0,0,0; ...
                  10*t,    10,-4,0,        0,0,0; ...
                  10.5*t,  12,-5,0,        0,0,0; ...
                  11.5*t,    9,-7,0,         0,0,0; ...
                  13*t,  6,-10,0,        0,0,0; ...
                  14*t,    3,-13,0,        0,0,0; ...
                  16*t,  0,-8,0,         0,0,0; ...
                  17*t,    0,-5,0,         180,0,0];    % Final position
elseif course == 1
%% square Path


               % Time,  Waypoint(x,y,z),Orientation
trajectoryInfo = [0,    0,-r,0,         0,0,0; ... % Initial position
                  0.010,  -r,0,0,         0,0,0; ...
                  0.020,    0,r,0,          0,0,0; ...
                  0.030,  r,0,0,          0,0,0; ...
                  0.040,    0,-r,0,         0,0,0;
                  0.050,  -r,0,0,         0,0,0];    % Final position
elseif   course == 2            
%% figure 8
                                 % Time,  Waypoint(x,y,z),Orientation
trajectoryInfo = [0,    0,0,0,         0,0,0; ... % Initial position
                  2,  r,-r,0,         0,0,0; ...
                  4,  r,r,0,          0,0,0; ...
                  6,  -r,-r,0,          0,0,0; ...
                  8,  -r,r,0,         0,0,0;
                  10,  0,0,0,         0,0,0];    % Final position
elseif  course == 3            %% circle Path


               % Time,  Waypoint(x,y,z),Orientation
trajectoryInfo = [0,    0,-r,0,         0,0,0; ... % Initial position
                  1,  -r,0,0,         0,0,0; ...
                  2,    0,r,0,          0,0,0; ...
                  3,  r,0,0,          0,0,0; ...
                  4,    0,-r,0,         0,0,0;
                  5,  -r,0,0,         0,0,0];    % Final position
else
%% 2 points
                                 % Time,  Waypoint(x,y,z),Orientation
trajectoryInfo = [0,    0,0,0,         0,0,0; ... % Initial position
                  0.1,  r,-r,0,      0,0,0; ...
                  0.2,  2*r,-2*r,0,         0,0,0];    % Final position
end
trajectory = waypointTrajectory(trajectoryInfo(:,2:4), ...
    'TimeOfArrival',trajectoryInfo(:,1), ...
    'Orientation',quaternion(trajectoryInfo(:,5:end),'eulerd','ZYX','frame'), ...
    'SampleRate',100);



%% plot points 
figure(1)
plot(trajectoryInfo(:,2),trajectoryInfo(:,3),'g.', 'MarkerSize', 10)
title('Position')
% axis([-length(trajectoryInfo),length(trajectoryInfo),-length(trajectoryInfo),length(trajectoryInfo)])
axis square
xlabel('X')
ylabel('Y')
grid on
hold on

orientationLog = zeros(toa(end)*trajectory.SampleRate,1,'quaternion');
%initialise as empty vectors
xpos =[];
ypos =[];
count = 1;
while ~isDone(trajectory)
   [currentPosition,orientationLog(count)] = trajectory();    
   plot(currentPosition(1),currentPosition(2),'bo')
   xpos = [xpos,currentPosition(1)];
   ypos = [ypos,currentPosition(2)];
   pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
   count = count+1;
end
plot(trajectoryInfo(:,2),trajectoryInfo(:,3),'g.', 'MarkerSize', 10)
hold off

Map = [xpos;ypos];
% save('map.mat','Map')
save('xpos.mat','xpos')
save('ypos.mat','ypos')

