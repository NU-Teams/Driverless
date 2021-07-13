clear
close all
clc

%% initalise path
data1 = load('xpos.mat');
data2 = load('ypos.mat');
path = [data1.xpos;data2.ypos]';

%handling track
data2 = load('Path.mat');
Array = data2.p;

% double the data in X column
x_extended = interp(Array(:,1),1);
% dobule the data in Y column
y_extended = interp(Array(:,2),1);
% new data containing all data
% path = [x_extended,  y_extended];
path = [Array(:,1),  Array(:,2)];
%% initalise robot model
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = deg2rad(90);
RobotEstimatedPose = [robotInitialLocation initialOrientation]';
lastpose  = RobotEstimatedPose;
velocity = 0;

%% setup controller constraints
controller.Waypoints = path;
controller.DesiredLinearVelocity = 6;
controller.MinLinearVelocity = 0.5;
controller.MaxAngularVelocity = deg2rad(100);
controller.LookaheadDistance = 1.5;
controller.lastAlpha = RobotEstimatedPose(3);
S = 0.001; % noise covarience to add to robot to check controller for disturbance rejection
%% Initialize the simulation loop

time = 1500;
sampleTime = 0.1;

vizRate = rateControl(1/sampleTime);
states = [];
% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize  = 5.5;

for t = 1:time

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, w, lookaheadPoint, a] = next_step(controller, RobotEstimatedPose);
    
    % Get the robot's velocity using controller inputs
    theta = RobotEstimatedPose(3);
    StateDot = [cos(theta) 0; sin(theta) 0; 0 1]*[v; w];

                                                               %added noise                                                                %                                                          
    % Update the current pose
    RobotEstimatedPose = RobotEstimatedPose + StateDot*sampleTime ;%+ sqrtm(S)*randn(length(StateDot),1); 
    
    
    velocity =  RobotEstimatedPose(1:2,:) - lastpose(1:2,:);        %need to chage estimated to sensor read actual velocity
    %Save States
    states = [states RobotEstimatedPose(1:2,:)];
    
    % print current velocity and angle, v = sqrt(x^2+y^2)
    fprintf('time = %d Estimated velocity = %.2f actual velocity = %.2f Angle = %.2f\n',t,norm(StateDot(1:2,:)),norm(velocity)*10,rad2deg(RobotEstimatedPose(3)))
    
  
    % Update the plot
    hold off

%   plot Path as grey road and smaller green path to show desired path
    plot(path(:,1), path(:,2),'.','Color', [0.5, 0.5, 0.5],'MarkerSize', 70) %grey Dots
    hold on
    plot(path(:,1), path(:,2),"w") %green line
    hold on 
    plot(lookaheadPoint(1),lookaheadPoint(2),'r*')
    hold on
    %plot controller angle heading
    plot([RobotEstimatedPose(1,:),RobotEstimatedPose(1,:)+2*cos(a+RobotEstimatedPose(3,:))],[RobotEstimatedPose(2,:),RobotEstimatedPose(2,:)+2*sin(a+RobotEstimatedPose(3,:))],'MarkerSize', 40)
    hold on
    plot(states(1,:),states(2,:),'b.','MarkerSize', 5)
    legend('road','Desired Path','Lookahead Point','controller angle heading','state trajectory','Location','southeast')
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [RobotEstimatedPose(1:2); 0];
    %calculate quaternion of robot angle around z axis
    plotRot = axang2quat([0 0 1 RobotEstimatedPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath","groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
%     plot(lookaheadPoint(1),lookaheadPoint(2),'r*')
    light;
    xlim([min(path(:,1))-15 max(path(:,1))+15])
    ylim([min(path(:,2))-15 max(path(:,2))+15])

    F(t) = getframe(gcf); %Frame for video  

    waitfor(vizRate);
    lastpose = RobotEstimatedPose;


end


%% create the video writer 
writerObj = VideoWriter('handling_track.avi');
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