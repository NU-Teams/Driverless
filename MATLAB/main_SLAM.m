% NOTES: It is assumed that an input is to be provided by the controller
% currently the solution does not interact a crontroller, so u will be
% generated ahead of time.


close all
clear all
clc

addpath('data','functions','models','estimation')

disp('Running EKF SLAM Example: SLAM with unknown correspondence & limited field of view')


rng(1)
%% OPTIONS:
 % Save data for analysis / debugging
SLAM.saveData = false; 

% Include noise to measurements
SLAM.addNoise = false;

% Use Mahalanobis [true] or Euclidean [false]
SLAM.mahalanobis = false;
SLAM.min_radius = 3.5; % radius [m]

% Animation options
SLAM.animate = true;
SLAM.saveVideo = false; % animate must also be true
SLAM.videoFileName = '../videos/EKFSLAM_limitedFOV_euclideanDistance_Noise_DeletingLandmarks1.avi';

if SLAM.animate
    % Setup animation
    figure(2)
    set(gcf, 'Position',  [1000, 200, 500, 500])
    colourSelect = [1 0 0 0.05];
    squareAxis = linspace(0,100,500);
%     lm = plot(L(:,1),L(:,2),'.','color', [0,0,0,0.2], 'linewidth', 1, 'markersize',4);
end

%% Load landmarks (simulation) & initial pose
SLAM.gt.initPose = [20; 30; 0]; % x, y, psi %% move to setup

%% Declare Models
% Vehicle Model. 
    % Examples: [robotModel2Wheels, simpleBicycleModel]
SLAM.vehicleModel = 'robotModel2Wheels';

% Track Option (track xy coordinates) & Data associated w/ track
SLAM.track = 'trackLoop_test1.mat';
SLAM.controllerInput = 'data_test1.mat';
SLAM.STM = 'forwardEulerMethod';

% Setup Simulation
SLAM = setup(SLAM);

% Initialise States
xp = SLAM.gt.x;
Pp = 10.*eye(SLAM.nx+SLAM.ny*SLAM.nLandmarks);
xf = 0; Pf = 0;
p = [];
t = 1;
while SLAM.active
    %% Generate Simulated Data
    % Generate States from [t-1] -> [t]
    if t ~= 1
        [x] = STM(SLAM,x,SLAM.gt.u(:,t));
    else
        x = xp;
    end
    % Generate Measurements, ground truth & simulated data
    [zgt, z] = getMeasurements_rangeBearing(x,SLAM.gt.landmarks,SLAM);
    z = zgt;
    % Add noise to the input signal
%     u = SLAM.gt.u(:,t) + 0.01*randn(SLAM.nu,1);
    u = SLAM.gt.u(:,t);

    %% SLAM
%         [muf,Pf,SLAM] = EKF_SLAM(SLAM, muf, Pf, SLAM.obs.u, SLAM.obs.z);
    [xf,Pf,xp,Pp] = EKF_SLAM_take2(SLAM, xp, Pp, u, z);

    %% Store, animate etc
    if SLAM.saveData
        %% Create vectors
        if t == 1
           % States Inputs & Measurements (static / no change in size)
           SLAM.data.x = [];
           SLAM.data.u = [];
           SLAM.data.z = []; % I lied this is dynamic
           
           % Mean & Covariance Matrices (dynamic / changing in size)
           SLAM.data.mu = [];
           SLAM.data.P  = [];
        end
        %% Resize Matrices
            % Resize Z ?
            if size(SLAM.obs.z,1) > size(SLAM.data.z,1)
                size_difference = size(SLAM.obs.z,1) - size(SLAM.data.z,1);
                SLAM.data.z = [SLAM.data.z; NaN(size_difference,size(SLAM.data.z,2))];
            end
            % Resize mu ?
            if size(xf,1) > size(SLAM.data.mu,1)
                size_difference = size(xf,1) - size(SLAM.data.mu,1);
                SLAM.data.mu = [SLAM.data.mu; NaN(size_difference,size(SLAM.data.mu,2))];
            end

            % Resize P ?
            if size(Pf,1) > size(SLAM.data.P,1)
                size_difference = size(Pf,1) - size(SLAM.data.P,1);
                SLAM.data.P = [SLAM.data.P, ...
                               NaN(size(SLAM.data.P,1),size_difference,size(SLAM.data.P,3));...
                               NaN(size_difference,size(SLAM.data.P,2),size(SLAM.data.P,3)), ...
                               NaN(size_difference,size_difference,size(SLAM.data.P,3))];
            end
            
        %% Add data to exisiting fields (ew, inefficient, I know)
        SLAM.data.x = [SLAM.data.x, SLAM.obs.x];
        SLAM.data.u = [SLAM.data.u, SLAM.obs.u];
        SLAM.data.z = [SLAM.data.z, SLAM.obs.z];
        SLAM.data.mu = [SLAM.data.mu, xf];
        
        % FIXME: I dunno if this is will work vvv Its a 3D vector
        SLAM.data.P  = [SLAM.data.P, Pf];
        
        % Store 
    end
    if SLAM.animate
       p = plotMap(xf, Pf, z, t, p, SLAM);
%         if t == 1
%             hhh = plot(x(1),x(2),'k*', 'markersize', 12)
%         else
%             set(hhh,'xdata',x(1),'ydata',x(2));
%         end
%         drawnow
        
    end    
    % increase timestep for next iteration
    t = t + 1;
    if t > size(SLAM.gt.u,2)
        SLAM.active = false;
    end
end


%% Overview
% "Come on and SLAM, if you want to jam" : Space Jame Theme Song
%
% Starting with an EKF SLAM utilising landmark mapping.
% 

%% Resources
% 

%% The Plan
% 1: Localisation from simulated data. (Simples. From LiDAR measurement)
    % Create a MAP (Is this just a point or do I declare a landmark??)
    % Create simulated LiDAR data, input data from scanNE (basically adds noise)
    % Input a trajectory through a map, collect simulated data.
    % Localise myself
    
% 2: Add noise to the above

% 3: Add landmarks to the states

% 4: Start Mapping w/ landmarks

% 5: ?

% 6: Profit
