% NOTES: It is assumed that an input is to be provided by the controller
% currently the solution does not interact a crontroller, so u will be
% generated ahead of time.


close all
clear all
clc

addpath('data','functions','models','estimation');
disp('Running EKF SLAM Example: SLAM with unknown correspondence & limited field of view');

% rng(1)
%% OPTIONS:
 % Save data for analysis / debugging
SLAM.saveData = false;

% Include noise to measurements
SLAM.addNoise = true;

% Use Mahalanobis [true] or Euclidean [false]
SLAM.mahalanobis = true;
SLAM.min_radius = 2; % radius [m]

% Animation options
SLAM.visualise = false;
SLAM.animate = true;
SLAM.saveVideo = true; % animate must also be true
SLAM.videoFileName = 'videos/EKFSLAM_robotModel2Wheels_LandmarkManagement_euclideanDistance1m_Noise.avi';

% Delete options
SLAM.delete = true;
SLAM.fileName = 'data/lmmv.bin';

try
    file = fopen(SLAM.fileName, 'w');
    lmmv = fwrite(file,[],'int8');
    fclose(file);
catch
    disp('no binary file');
end

%% Load landmarks (simulation) & initial pose
SLAM.gt.initPose = [0; 0; 0]; % x, y, psi %% move to setup

%% Declare Models
% Vehicle Model. 
    % Examples: [robotModel2Wheels, simpleBicycleModel, motion_model_velocity]
SLAM.vehicleModel = 'robotModel2Wheels';

% Track Option (track xy coordinates) & Data associated w/ track
SLAM.track = 'data_robotModel2Wheels_loop1.mat';
SLAM.controllerInput = 'data_robotModel2Wheels_loop1.mat';
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
    if SLAM.addNoise
        % Add noise to the input signal
        u = SLAM.gt.u(:,t) + 0.01*randn(SLAM.nu,1);
    else
        % Use Ground truth values
        z = zgt;    
        u = SLAM.gt.u(:,t);
    end
    %% SLAM
    if t == 1
        xf = xp;
        Pf = Pp;
    else
        tic
        [xf,Pf] = EKF_SLAM(SLAM, xf, Pf, u, z);
        toc
    end
    
    if SLAM.animate
        [p, SLAM] = plotMap(xf, Pf, z, t, p, SLAM);        
        if SLAM.saveVideo
            F(t) = getframe(gcf);
        end

    end 
    
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
 
    % increase timestep for next iteration
    t = t + 1;
    if t > size(SLAM.gt.u,2)
        SLAM.active = false;
    end
end
if SLAM.saveVideo
    writerObj = VideoWriter(SLAM.videoFileName);
    writerObj.FrameRate = 20; % sets the fps

    % open the video writer
    open(writerObj);

    % write the frames to the video
    for t = 1:length(F)
        % convert the image to a frame
        frame = F(t);
        writeVideo(writerObj, frame)
    end
    close(writerObj);
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
