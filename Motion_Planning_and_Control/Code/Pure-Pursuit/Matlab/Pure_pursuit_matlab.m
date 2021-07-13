clear
close all
clc

%Simulation for Pure-Pursuit controller

%% initalise path
%Aidan Mc Track
%to generate this data run the waypoint generation matlab script
data1 = load('xpos.mat');
data2 = load('ypos.mat');
path = [data1.xpos;data2.ypos]';

%AMZ Track
%you can find this track on
%https://github.com/alexliniger/MPCC
% load track2.mat

%plot track
% figure(1);
% plot(track2.outer(1,:),track2.outer(2,:),'k')
% hold on
% plot(track2.inner(1,:),track2.inner(2,:),'k')
% plot(track2.center(1,:),track2.center(2,:),'.r')

%Scale AMZ path
% path = 10*[track2.center]';


%% initalise robot model
% for implementation this will have to be a seperate script that takes in
% data and localises the robot.
robotInitialLocation = path(5,:);
robotGoal = path(end,:);
% initialOrientation = deg2rad(290);  %angle for AMZ track
initialOrientation = deg2rad(90);     %angle for Aidan track
initialwheelAngle = deg2rad(0);       %initialise wheel angle as 0
RobotEstimatedPose = [robotInitialLocation initialOrientation 0]';
lastpose  = RobotEstimatedPose;         
velocity = 0;
l = 3;  %%length of bycicle model
% robot  = bicycleKinematics("VehicleInputs","VehicleSpeedHeadingRate","MaxSteeringAngle",pi/8);
% robot  = ackermannKinematics;

%% setup controller constraints
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MinLinearVelocity = 3;
controller.MaxAngularVelocity = pi;
controller.LookaheadDistance = 1;  %1 for aidan track
controller.Wb = 2.9;
d = controller.LookaheadDistance;
vel = controller.DesiredLinearVelocity;
controller.lastAlpha = RobotEstimatedPose(3);
a0 = 0;
S = 0.001; % noise covarience to add to robot to check controller for disturbance rejection
%% Initialize the simulation loop

time = 1500;
sampleTime = 0.1;
t = 1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize  = 1.5;

%for t = 1:time
while(1)

    % Compute the controller outputs, i.e., the inputs to the robot
    [v,w, lookaheadPoint, alpha] = next_step(controller, RobotEstimatedPose([1 2 3]));

    % Get the robot's velocity using controller inputs
    phi = RobotEstimatedPose(4);
    theta = RobotEstimatedPose(3);
    
    angle = 100*atan(2*sin(alpha)/d*v);
    delta = -(a0 - angle);
    a0 = angle;

    StateDot = [cos(theta) 0; sin(theta) 0;tan(phi)/l 0;0 1]*[v; delta];
                                                             %                                                          
    % Update the current pose
    RobotEstimatedPose = RobotEstimatedPose + StateDot*sampleTime ;%+ sqrtm(S)*randn(length(StateDot),1); 
    
    
    velocity =  RobotEstimatedPose(1:2,:) - lastpose(1:2,:);        %need to chage estimated to sensor read actual velocity

    
    % print current velocity and angle, v = sqrt(x^2+y^2)
    fprintf('time = %d Estimated velocity = %.2f actual velocity = %.2f Angle = %.2f\n',t,norm(StateDot(1:2,:)),norm(velocity)*10,rad2deg(RobotEstimatedPose(3)))
    
  
    % Update the plot
    hold off

%   plot Path as grey road and smaller green path to show desired path
    plot(path(:,1), path(:,2),'.','Color', [0.5, 0.5, 0.5],'MarkerSize', 70) %grey Dots
    hold on
    plot(path(:,1), path(:,2),"g") %green line
    hold on 
    plot(lookaheadPoint(1),lookaheadPoint(2),'r*')
    hold on
    %plot controller angle heading
    plot([RobotEstimatedPose(1,:),RobotEstimatedPose(1,:)+2*cos(alpha+RobotEstimatedPose(3,:))],[RobotEstimatedPose(2,:),RobotEstimatedPose(2,:)+2*sin(alpha+RobotEstimatedPose(3,:))],'MarkerSize', 40)
    hold on
%     plot([RobotEstimatedPose(1,:),RobotEstimatedPose(1,:)+2*cos(delta+RobotEstimatedPose(3,:))],[RobotEstimatedPose(2,:),RobotEstimatedPose(2,:)+2*sin(delta+RobotEstimatedPose(3,:))],'y')
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [RobotEstimatedPose(1:2); 0];
    %calculate quaternion of robot angle around z axis
    plotRot = axang2quat([0 0 1 RobotEstimatedPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath","groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);

    light;
    xlim([min(path(:,1))-5 max(path(:,1))+5])
    ylim([min(path(:,2))-5 max(path(:,2))+5])
    F(t) = getframe(gcf); %Frame for video  
    waitfor(vizRate);
    lastpose = RobotEstimatedPose;
    t = t + 1;
end


%% create the video writer 
writerObj = VideoWriter('Matlab_Pure_pursuit.avi');
writerObj.FrameRate = 50; % sets the fps

% open the video writer
open(writerObj);
% write the frames to the video
for t = 1:length(F)
    % convert the image to a frame
    frame = F(t);
    writeVideo(writerObj, frame)
end
close(writerObj);