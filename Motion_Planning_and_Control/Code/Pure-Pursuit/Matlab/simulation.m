clear
close all
clc

%Simulation for Pure-Pursuit controller Designed By UON Driverless Aidan McEnearney 2020

%% initalise path
%AidanMc Track
data1 = load('xpos.mat');
data2 = load('ypos.mat');
path = [data1.xpos;data2.ypos]';

% %AMZ Tack
% load track2.mat
% 
% figure(1);
% plot(track2.outer(1,:),track2.outer(2,:),'k')
% hold on
% plot(track2.inner(1,:),track2.inner(2,:),'k')
% plot(track2.center(1,:),track2.center(2,:),'.r')
% 
% path = 10*[track2.center]';


%% initalise robot model
% for implementation this will have to be a seperate script that takes in
% data and localises the robot.
%set x and y cordinates to be just off path
robotInitialLocation = path(1,:);
%start robot point up
initialOrientation = deg2rad(90);

initialwheelAngle = deg2rad(0);

state = [robotInitialLocation initialOrientation 0]';


% state.x = robotInitialLocation(1);
% state.y = robotInitialLocation(2);
% state.yaw = initialOrientation;
% state.v = 0;
% state.phi = 0;

l = 3;  %%length of bycicle model


%% setup controller constraints
% controller.Waypoints = path;
controller.map = path;
% controller.DesiredLinearVelocity = 0.5;
% controller.MinLinearVelocity = 3;
% controller.MaxAngularVelocity = pi;
% controller.LookaheadDistance = 0.010;
controller.Wb = 2.9;
controller.oldNearestPoint = 0;
controller.k = 0.1; %gain for look ahead distance
controller.Lfc = 2; %Ideal lookahead distance



%% Initialize the simulation loop

time = 1500;
sampleTime = 0.1;
dt = sampleTime;
pt.x=[];
pt.y=[];

vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize  = 1.5;
for t = 1:time
    %proportional controller
%     a = proportional(controller,state.v);
    a = 3;
    % Compute the controller outputs, i.e., the inputs to the robot
    [delta, index] = pure_pursuit_steer_control(state, path, controller);
    
    [state, pt] = state_update(state, delta, 1, pt);
     % Update the plot
    hold off

%   plot Path as grey road and smaller green path to show desired path
    plot(path(:,1), path(:,2),'.','Color', [0.5, 0.5, 0.5],'MarkerSize', 70) %grey Dots
    hold on
    plot(path(:,1), path(:,2),"g") %green line
%     hold on 
%     plot(index(1),index(2),'r*')
    hold on
    %plot controller angle heading
    plot([state(1,:),state(1,:)+2*cos(a+state(3,:))],[state(2,:),state(2,:)+2*sin(a+state(3,:))],'MarkerSize', 40)
    hold on
%     plot([RobotEstimatedPose(1,:),RobotEstimatedPose(1,:)+2*cos(delta+RobotEstimatedPose(3,:))],[RobotEstimatedPose(2,:),RobotEstimatedPose(2,:)+2*sin(delta+RobotEstimatedPose(3,:))],'y')
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [state(1:2); 0];
    %calculate quaternion of robot angle around z axis
    plotRot = axang2quat([0 0 1 state(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath","groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);

    light;
    xlim([min(path(:,1))-5 max(path(:,1))+5])
    ylim([min(path(:,2))-5 max(path(:,2))+5])
    F(t) = getframe(gcf); %Frame for video  
    waitfor(vizRate);
    lastpose = state;

    plot_car(state, path, index)
    
    fprintf('time = %d Angle = %.2f x = %.2f y = %.2f\n',t,rad2deg(state(3)),state(1),state(2))
    

%     plot(track2.outer(1,:),track2.outer(2,:),'k')
%     hold on
%     plot(track2.inner(1,:),track2.inner(2,:),'k')
%     plot(track2.center(1,:),track2.center(2,:),'.r')
%     plot(pt.x(:),pt.y(:),'*b')
%     drawnow
 

end


% %% create the video writer 
% writerObj = VideoWriter('BicycleModel.avi');
% writerObj.FrameRate = 50; % sets the fps
% 
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for t = 1:length(F)
%     % convert the image to a frame
%     frame = F(t);
%     writeVideo(writerObj, frame)
% end
% close(writerObj);



function [state,pt] = state_update(state, delta, a, pt)
%     dt = 1;
    Wb = 3;
%     state.x = state.v*cos(state.yaw)*dt;
%     state.y = state.v*sin(state.yaw)*dt;
%     state.yaw = state.v/Wb*tan(delta)*dt;
%     state.v = a * dt;
    theta = state(3);
    phi = state(4);
    
    stateDot = [cos(theta) 0; sin(theta) 0;tan(phi)/Wb 0;0 1]*[0.1; delta];
    state = state + stateDot;
    pt.x = [pt.x, state(1)];
    pt.y = [pt.y, state(2)];

end

function [] = plot_car(state, path, index)


% %   plot Path as grey road and smaller green path to show desired path
%     plot(path(:,1), path(:,2),'.','Color', [0.5, 0.5, 0.5],'MarkerSize', 70) %grey Dots
%     hold on
%     plot(path(:,1), path(:,2),"g") %green line
%     xlim([-20 20])
%     ylim([-20 20])
%     L = 1 ; B = 2 ;
% %     car = [0. 0. ; L 0. ; L B ; 0. B; 0 0.] ;
%     car = [-L/2 -B/2;L/2 -B/2;L/2 B/2;-L/2 B/2;-L/2 -B/2];
%     angles = atan2(car(:,2),car(:,1))+ deg2rad(state(3));
%     r = sqrt(car(:,1).^2+car(:,2).^2);
%     car2 = [r.*cos(angles),r.*sin(angles)];
%     plot(state(1)+ car2(:,1),state(2)+ car2(:,2),'b')
%     hold on
    plot(path(index,1),path(index,2),'*r')
    drawnow
end
